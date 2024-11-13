#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::can::filter::{ListEntry16, BankConfig};
use embassy_stm32::can::util::NominalBitTiming;
use embassy_stm32::exti::ExtiInput;
use core::num::{NonZeroU16, NonZeroU8}; //used for can bit timings
use embassy_stm32::{adc, bind_interrupts, can, gpio, init, peripherals::{self, *}, rcc, timer, Config};
use embassy_stm32::adc::{AdcChannel, Adc, AnyAdcChannel};
use embassy_stm32::can::{filter, Can, Frame, StandardId};
use embassy_stm32::can::enums::TryReadError;


use embassy_stm32::peripherals::ADC4;
use embassy_stm32::time::{mhz, hz};
use embassy_time::Timer;

//mutex & multi-tasking includes
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use static_cell::StaticCell;
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

//////////////////////////////////Send Msg Data
static TORQUE_DATA: AtomicU32 = AtomicU32::new(0);
static FAILURES: AtomicU32 = AtomicU32::new(0);

/////////////////////////////////Recive Msg Data
static PDM_STATES: AtomicU32 = AtomicU32::new(0);
static PDM_CURRENTS: AtomicU32 = AtomicU32::new(0);

/////////////////////////////////Can Addresses: TX
const CANID_MC_TORQUE: u16 = 0x192;              // Address to send message to MC
////////////////////////////////Can Adresses: RX
const CANID_BRAKE: u16 = 0x245;              // Send Brake on can id
const CANID_FAILURES: u16 = 0x250;           // Send failures on can id
const CANID_ORIONBMS: u16 = 0x3B;            // Receive OrionBMS Main Channel
const CANID_ORIONBMS2: u16 = 0x6B2;          // Receive OrionBMS secondary channel
const CANID_PDMRESET: u16 = 0x522;           // Receive PDM Water Pump Current and diag
const CANID_PDMSTATUS: u16 = 0x520;          // Receive pdm vehicle states and faults
const CANID_M150_TEMPERATURE: u16 = 0x502;   //Receive tractive system temps from m150

// Mutex type to safely share CAN bus between tasks
type can_bus_mut_type = Mutex<ThreadModeRawMutex, can::Can<'static>>;

#[embassy_executor::task]
async fn can_write_task(bus: &'static can_bus_mut_type) {
    loop {
        Timer::after_millis(50).await;
        let can_frame: Frame = Frame::new_data(StandardId::new(CANID_MC_TORQUE).unwrap(), &u32tou8array(&TORQUE_DATA.load(Ordering::Relaxed))).unwrap();
        let mut bus_unlock = bus.lock().await; //waits for canbus peri to be free
        bus_unlock.write(&can_frame).await; 
    }
}

#[embassy_executor::task]
async fn can_read_task(bus: &'static can_bus_mut_type) {
    info!("CAN Read Task Begin");
    loop {
        {
        let mut bus_unlock = bus.lock().await; //waits for canbus peri to be free
        // let try_read = bus_unlock.try_read(); //try read
        loop{
            let try_read = bus_unlock.try_read(); //try read
            if try_read.is_ok() == true{
                let readframe = try_read.unwrap().frame;
                ////////////////////////////////PDM STATUS RECIVE
                if *readframe.header().id() == StandardId::new(CANID_PDMSTATUS).unwrap().into(){
                    let data: &[u8] = readframe.data();
                    PDM_STATES.store( u8arraytou32(data), Ordering::Relaxed)
                }
                ////////////////////////////////
            }else{
                break;
            }
        }
        }
        Timer::after_micros(100).await;
    }
}

#[embassy_executor::task]
async fn sensor_task(mut adc4: Adc<'static, ADC4>, 
                    mut adc3: Adc<'static, ADC3>,  
                    mut ain1: AnyAdcChannel<ADC4>, 
                    mut ain2: AnyAdcChannel<ADC3>) 
{
    loop {
        Timer::after_millis(100).await;
        let ain1_reading = adc4.read(&mut ain1).await;// Read the ADC value from the specified pin
        //info!("AIN1 reading: {}", ain1_reading);
        let ain2_reading = adc3.read(&mut ain2).await;
        //info!("AIN2 reading: {}", ain2_reading);
        TORQUE_DATA.store(ain2_reading as u32, Ordering::Relaxed);
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
    
    let canfiltbank1 = BankConfig::List16([ListEntry16::data_frames_with_id(StandardId::new(CANID_BRAKE).unwrap()), 
                                                        ListEntry16::data_frames_with_id(StandardId::new(CANID_FAILURES).unwrap()),
                                                        ListEntry16::data_frames_with_id(StandardId::new(CANID_ORIONBMS).unwrap()),
                                                        ListEntry16::data_frames_with_id(StandardId::new(CANID_PDMSTATUS).unwrap())]);

    let canfiltbank2 = BankConfig::List16([ListEntry16::data_frames_with_id(StandardId::new(CANID_PDMRESET).unwrap()), 
                                                        ListEntry16::data_frames_with_id(StandardId::new(CANID_ORIONBMS2).unwrap()),
                                                        ListEntry16::data_frames_with_id(StandardId::new(CANID_PDMSTATUS).unwrap()),
                                                        ListEntry16::data_frames_with_id(StandardId::new(CANID_M150_TEMPERATURE).unwrap())]);


    car_bus.modify_filters().enable_bank(0, can::Fifo::Fifo0, canfiltbank1);
    car_bus.modify_filters().enable_bank(0, can::Fifo::Fifo0, canfiltbank2);
    
    car_bus.enable().await;                                                                       //enable can
    static CAN_BUS_CELL: StaticCell<can_bus_mut_type> = StaticCell::new();//static cell lets us initialize at runtime into memory reserved at compile time
    let car_bus_mutex = CAN_BUS_CELL.init(Mutex::new(car_bus)); //static cell used to init a new mutex of canbus peri

    ////////////////////////////Configure ADC peripherals
    let adc3 = Adc::new(p.ADC3, Adc3Irqs);
    let adc4 = Adc::new(p.ADC4, Adc4Irqs);

    ///////////////////////////Configure input button with interupt
    let mut rtd_button: ExtiInput<'_> = ExtiInput::new(p.PE2, p.EXTI2, gpio::Pull::None);

    //////////////////////////Configure RTD sound pin
    let rtd_sound_pin = Some(timer::simple_pwm::PwmPin::new_ch1(p.PA11, gpio::OutputType::PushPull));
    let mut buzzer_button = timer::simple_pwm::SimplePwm::new(p.TIM4, rtd_sound_pin, Option::None, Option::None, Option::None, hz(261), timer::low_level::CountingMode::CenterAlignedBothInterrupts);
    buzzer_button.ch1().enable();
    Timer::after_millis(200).await; //try beep
    buzzer_button.ch1().disable();

    // Spawn tasks
    spawner.spawn(can_write_task(car_bus_mutex)).unwrap();
    spawner.spawn(can_read_task(car_bus_mutex)).unwrap();
    spawner.spawn(sensor_task(adc4, adc3, p.PE8.degrade_adc(), p.PE9.degrade_adc())).unwrap();

    // Main loop: state transitions
    loop {
        //Check if button got pressed
        rtd_button.wait_for_rising_edge().await;
        info!("button rising edge detected");
        Timer::after_millis(100).await;
        if rtd_button.is_high() && 
        !RTD_STATE.load(Ordering::Relaxed) && 
        (u32tou8array(&PDM_STATES.load(Ordering::Relaxed))[5] == 255) //check what pdm puts for booleans
        {
            //todo: the beeping thing
            RTD_STATE.store(true, Ordering::Relaxed);
        }
    }
}

pub fn u32tou8array(data: &u32) -> [u8; 4] {
    let mut output = [0; 4];
    output[..4].copy_from_slice(&data.to_le_bytes()); //copies the memory representation in slices, little endian byte order
    output //returns result
}

pub fn u8arraytou32(input_array: &[u8]) -> u32{
    let mut output: u32 = 0;
    let mut i = 0;
    for byte in input_array {
        output = ((*byte as u32) << i*8) | output;
        i+=1;
    };
    output //return
}
