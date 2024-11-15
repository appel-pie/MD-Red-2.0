#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::can::filter::{ListEntry16, BankConfig};
use embassy_stm32::can::util::NominalBitTiming;
use embassy_stm32::exti::ExtiInput;
use core::num::{NonZeroU16, NonZeroU8}; //used for can bit timings
use embassy_stm32::{adc, bind_interrupts, can, gpio, init, peripherals::{self, *}, rcc, Config};
use embassy_stm32::adc::{AdcChannel, Adc, AnyAdcChannel};
use embassy_stm32::can::{Can, Frame, StandardId};
use bitbybit::bitfield;
use embassy_stm32::peripherals::ADC4;
use embassy_stm32::time::mhz;
use embassy_time::Timer;
use arbitrary_int::{u1, u13};

//mutex & multi-tasking includes
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use static_cell::StaticCell;
use embassy_sync::pubsub::PubSubChannel;

//logging
use {defmt_rtt as _, panic_probe as _,}; 

/////////////////interrupt bindings
bind_interrupts!(struct CanIrqs {
    USB_LP_CAN_RX0 => can::Rx0InterruptHandler<peripherals::CAN>;
    CAN_RX1 => can::Rx1InterruptHandler<peripherals::CAN>;
    CAN_SCE => can::SceInterruptHandler<peripherals::CAN>;
    USB_HP_CAN_TX => can::TxInterruptHandler<peripherals::CAN>;
});

bind_interrupts!(struct Adc4Irqs {
    ADC4 => adc::InterruptHandler<ADC4>;
});

bind_interrupts!(struct Adc3Irqs {
    ADC3 => adc::InterruptHandler<ADC3>;
});

/////////////////////////////////Firmware global state data
static RTD_STATE: AtomicBool = AtomicBool::new(false);

/////////////////////////////////Recive Msg Data
static PDM_STATES: PubSubChannel<ThreadModeRawMutex, u64, 2, 2, 1> = PubSubChannel::new();

////////////////////////////////Send Msg Data
static PM150_COMMAND: PubSubChannel<ThreadModeRawMutex, u64, 2, 1, 2> = PubSubChannel::new();

/////////////////////////////////Can Addresses: TX
const CANID_MC_TORQUE: u16 = 0x192;              // Address to send message to MC
const CANID_BRAKE: u16 = 0x245;              // Send Brake on can id
////////////////////////////////Can Adresses: RX

const CANID_FAILURES: u16 = 0x250;           // Send failures on can id
const CANID_ORIONBMS: u16 = 0x3B;            // Receive OrionBMS Main Channel
const CANID_ORIONBMS2: u16 = 0x6B2;          // Receive OrionBMS secondary channel
const CANID_PDMDATA: u16 = 0x522;           // Receive PDM Water Pump Current and diag
const CANID_PDMSTATUS: u16 = 0x520;          // Receive pdm vehicle states and faults
const CANID_M150_TEMPERATURE: u16 = 0x502;   //Receive tractive system temps from m150

// Mutex type to safely share CAN bus between tasks
type CanBusMutType = Mutex<ThreadModeRawMutex, can::Can<'static>>;

#[embassy_executor::task]
async fn can_write_task(bus: &'static CanBusMutType) {
    info!("CAN Write Task Begin");
    let mut torque_command_channel = PM150_COMMAND.subscriber().unwrap();
    loop {
        Timer::after_millis(50).await;
        let motor_broadcast = u64tou8array(&torque_command_channel.next_message_pure().await);
        let can_frame: Frame = Frame::new_data(StandardId::new(CANID_MC_TORQUE).unwrap(), &motor_broadcast).unwrap();
        {   //make sure mutex lock scope is reduced
            let mut bus_unlock = bus.lock().await; //waits for canbus peri to be free
            bus_unlock.write(&can_frame).await; 
        }
    }
}

#[embassy_executor::task]
async fn can_read_task(bus: &'static CanBusMutType) {
    info!("CAN Read Task Begin");
    let pdm_states_publish = PDM_STATES.publisher().unwrap();
    loop {
        { //nested for mutex lock scope: timer is outside 
        let mut bus_unlock = bus.lock().await; //waits for canbus peri to be free
        loop{
            let try_read = bus_unlock.try_read(); //try read
            if try_read.is_ok() == true{
                let readframe = try_read.unwrap().frame;
                ////////////////////////////////PDM STATUS RECIVE
                if *readframe.header().id() == StandardId::new(CANID_PDMSTATUS).unwrap().into(){
                    pdm_states_publish.publish_immediate(u8arraytou64( readframe.data()));
                    info!("pdm state writted");
                    if RTD_STATE.load(Ordering::Relaxed) == true && PdmStatesStruct::new_with_raw_value(u8arraytou64(readframe.data())).hv_precharged() == 0{
                        RTD_STATE.store(false, Ordering::Relaxed); 
                        info!("shutdown detected");
                    }//shutdown detected: immediate rtd set low. This is because i dont want to write some interrup and the other state transition data is all on button pressed interrupt.
                }else{
                    continue;
                }
            }else{
                break; //breaks inner loop -> mutex dropped -> async wait
            }
        }
        }
        Timer::after_millis(1).await;
    }
}

#[embassy_executor::task]
async fn sensor_task(mut adc4: Adc<'static, ADC4>, 
                    mut adc3: Adc<'static, ADC3>,  
                    mut ain1: AnyAdcChannel<ADC4>, 
                    mut ain2: AnyAdcChannel<ADC3>) 
{
let mc_torque_channel = PM150_COMMAND.publisher().unwrap();
let mut command_message = McMessageStruct::default();
let mut brake_throttle_fault = false;
mc_torque_channel.publish_immediate(command_message.raw_value);
    loop {
        Timer::after_millis(50).await;
        let ain1_reading = adc4.read(&mut ain1).await;// Read the ADC value from the specified pin
        //info!("AIN1 reading: {}", ain1_reading);
        let ain2_reading = adc3.read(&mut ain2).await;
        //info!("AIN2 reading: {}", ain2_reading);
        command_message = command_message.with_torque_command(((ain1_reading + ain2_reading/2)) as i16);
        if RTD_STATE.load(Ordering::Relaxed) && !brake_throttle_fault{
            command_message = command_message.with_inverter_enable(true.into()); //enable inverter
        }else{
            command_message = command_message.with_inverter_enable(false.into()); //disable inverter
        }
        mc_torque_channel.publish_immediate(command_message.raw_value);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Main entry");

    // Initialize and create handle for device peripherals
    let mut config = Config::default();
    {
        config.rcc.hse = Some(rcc::Hse {freq: mhz(32), mode: rcc::HseMode::Oscillator});
        config.rcc.hsi = false; //disable internal clock 
        config.rcc.pll = Some(rcc::Pll {src: rcc::PllSource::HSE, prediv: rcc::PllPreDiv::DIV8, mul: rcc::PllMul::MUL16});
        config.rcc.sys = rcc::Sysclk::HSE;
        config.rcc.ahb_pre =  rcc::AHBPrescaler::DIV4; 
        config.rcc.adc =  rcc::AdcClockSource::Pll(rcc::AdcPllPrescaler::DIV4); 
        config.rcc.adc34 =  rcc::AdcClockSource::Pll(rcc::AdcPllPrescaler::DIV4);
    }
    let p = init(config);
    info!("Configured");

    /////////////////////////////Configure CAN Peripheral
    let mut car_bus: Can<'static> = Can::new(p.CAN, p.PD0, p.PD1, CanIrqs);
    let canconfig = car_bus.modify_config();
    canconfig.set_bit_timing(NominalBitTiming {
        prescaler: NonZeroU16::new(1).unwrap(),
        seg1: NonZeroU8::new(5).unwrap(),
        seg2: NonZeroU8::new(2).unwrap(),
        sync_jump_width: NonZeroU8::new(8).unwrap(),
    });
    //car_bus.modify_filters().enable_bank(0, can::Fifo::Fifo0, can::filter::Mask32::accept_all()); //set can filter
    //let filterconfig = filter::Mask16::frames_with_std_id(can::StandardId::new(CANID_BRAKE).unwrap(), can::StandardId::new(0x7FF).unwrap())
    
    let canfiltbank1 = BankConfig::List16([ListEntry16::data_frames_with_id(StandardId::new(CANID_FAILURES).unwrap()),
                                                        ListEntry16::data_frames_with_id(StandardId::new(CANID_ORIONBMS).unwrap()),
                                                        ListEntry16::data_frames_with_id(StandardId::new(CANID_ORIONBMS2).unwrap()),
                                                        ListEntry16::data_frames_with_id(StandardId::new(CANID_PDMDATA).unwrap())]);

    let canfiltbank2 = BankConfig::List16([ListEntry16::data_frames_with_id(StandardId::new(CANID_PDMSTATUS).unwrap()),
                                                        ListEntry16::data_frames_with_id(StandardId::new(CANID_M150_TEMPERATURE).unwrap()), 
                                                        ListEntry16::data_frames_with_id(StandardId::ZERO), //placehodlerd
                                                        ListEntry16::data_frames_with_id(StandardId::ZERO)]);


    car_bus.modify_filters().enable_bank(0, can::Fifo::Fifo0, canfiltbank1);
    car_bus.modify_filters().enable_bank(0, can::Fifo::Fifo0, canfiltbank2);
    car_bus.enable().await;                                                                       //enable can

    static CAN_BUS_CELL: StaticCell<CanBusMutType> = StaticCell::new();//static cell lets us initialize at runtime into memory reserved at compile time
    let car_bus_mutex = CAN_BUS_CELL.init(Mutex::new(car_bus)); //static cell used to init a new mutex of canbus peri

    ////////////////////////////Configure ADC peripherals
    let adc3 = Adc::new(p.ADC3, Adc3Irqs);
    let adc4 = Adc::new(p.ADC4, Adc4Irqs);

    ///////////////////////////Configure input button with interupt
    let mut rtd_button: ExtiInput<'_> = ExtiInput::new(p.PB9, p.EXTI9, gpio::Pull::None);

    ////////////////////////Configure RTD sound pin: pwm not required
    let mut beep_pin = gpio::Output::new(p.PB8,gpio::Level::Low, gpio::Speed::Medium);

    beep_pin.set_high();
    Timer::after_millis(50).await;
    beep_pin.set_low();

    // Spawn tasks
    spawner.spawn(can_write_task(car_bus_mutex)).unwrap();
    spawner.spawn(can_read_task(car_bus_mutex)).unwrap();
    spawner.spawn(sensor_task(adc4, adc3, p.PE8.degrade_adc(), p.PE9.degrade_adc())).unwrap();

    let mut pdmdata = PDM_STATES.subscriber().unwrap();
    let mut pdmstates =  PdmStatesStruct::default();

    loop {
        //Check if button got pressed
        rtd_button.wait_for_falling_edge().await;
        info!("button rising edge detected");
        let test = pdmdata.try_next_message_pure();
        if test.is_some(){
            pdmstates = PdmStatesStruct::new_with_raw_value(test.unwrap());
        }
        Timer::after_millis(500).await; //button press for 1/5 of a second
        if rtd_button.is_low() &&       //button still press
            !RTD_STATE.load(Ordering::Relaxed) &&  //not already rtd
            pdmstates.hv_precharged() != 0 //if Precharged (can message from pdm)
        {
            //todo: the beeping thing
            RTD_STATE.store(true, Ordering::Relaxed);
            beep_pin.set_high();
            Timer::after_millis(500).await;
            beep_pin.set_low();
            Timer::after_millis(100).await;
            beep_pin.set_high();
            Timer::after_millis(500).await;
            beep_pin.set_low();
            Timer::after_millis(100).await;
            beep_pin.set_high();
            Timer::after_millis(500).await;
            beep_pin.set_low();

        }else if rtd_button.is_low() && RTD_STATE.load(Ordering::Relaxed){
            RTD_STATE.store(false, Ordering::Relaxed);
            beep_pin.set_high();
            Timer::after_millis(500).await;
            beep_pin.set_low();
        }

    }
}

pub fn u64tou8array(data: &u64) -> [u8; 8] {
    let mut output = [0; 8];
    output[..8].copy_from_slice(&data.to_le_bytes()); //copies the memory representation in slices, little endian byte order
    output //returns result
}

pub fn u8arraytou64(input_array: &[u8]) -> u64{
    let mut output: u64 = 0;
    let mut i: u8 = 0;
    for byte in input_array {
        output = ((*byte as u64) << i*8) | output;
        i+=1;
    };
    output //return
}

pub fn i16tou8array(data: &i16, offset_bytes: u8) -> [u8; 8] {
    let mut output = [0; 8];
    let temp = *data as i64;
    let temp = temp <<8*offset_bytes;
    output[..8].copy_from_slice(&temp.to_le_bytes()); //copies the memory representation in slices, little endian byte order
    output //returns result
}

#[bitfield(u64, default = 0)]
struct PdmStatesStruct {
    #[bits(0..=7, rw)]
    hvs_bms_fault: u8,

    #[bits(8..=15, rw)]
    hvs_pdoc_fault: u8,

    #[bits(16..=23, rw)]
    imd_imd_fault: u8,

    #[bits(24..=31, rw)]
    hvs_imd_fault: u8,

    #[bits(32..=39, rw)]
    standby: u8,

    #[bits(40..=47, rw)]
    hv_precharged: u8,

    #[bits(48..=55, rw)]
    bcm_control: u8,

    #[bits(56..=63, rw)]
    shutdown_lock: u8,
}

#[bitfield(u64, default = 4096)] //0000 0000 0000 0000 0001 0000 0000 0000 0000 (inverter forward: byte 4 =1)
struct McMessageStruct {
    #[bits(0..=15, rw)] /////I think this chip will do little endian right?
    torque_command: i16,

    #[bits(16..=31, rw)]
    speed_command: i16,

    #[bits(32..=39, rw)]
    forwards_direction: u8,

    #[bit(40, rw)] //byte 5, bits 0, 1, 2
    inverter_enable: u1,

    #[bit(41, rw)]
    inverter_discharge: u1,

    #[bit(42, rw)]
    speed_mode_enable: u1,

    #[bits(43..=55, rw)]
    unused: u13,

    #[bits(56..=63, rw)]
    shutdown_lock: u8,
}