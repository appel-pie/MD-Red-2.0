#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::can::util::NominalBitTiming;
use core::num::{NonZeroU16, NonZeroU8}; //used for can bit timings
use embassy_stm32::{adc, init, peripherals::*, peripherals, bind_interrupts, can, Config, rcc, gpio};
use embassy_stm32::adc::{AdcChannel, Adc, AnyAdcChannel};
use embassy_stm32::can::{Frame, StandardId, Can};
use embassy_stm32::can::enums::TryReadError;

use embassy_stm32::peripherals::ADC4;
use embassy_stm32::time::mhz;
use embassy_time::Timer;

//mutex & multi-tasking includes
use core::sync::atomic::{AtomicU32, Ordering};
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

//////////////////////////////////static variables
static CAN_MESSAGE: AtomicU32 = AtomicU32::new(0);

// Mutex type to safely share CAN bus between tasks
type can_bus_mut_type = Mutex<ThreadModeRawMutex, can::Can<'static>>;

#[embassy_executor::task]
async fn can_write_task(bus: &'static can_bus_mut_type) {
    info!("CAN Write Task Begin");
    let id = StandardId::new(100).unwrap();
    loop {
        Timer::after_millis(50).await;
        let can_frame: Frame = Frame::new_data(id, &u32tou8array(&CAN_MESSAGE.load(Ordering::Relaxed))).unwrap();
        let mut bus_unlock = bus.lock().await; //waits for canbus peri to be free
        bus_unlock.write(&can_frame).await; 
    }
}

#[embassy_executor::task]
async fn can_read_task(bus: &'static can_bus_mut_type) {
    info!("CAN Read Task Begin");
    loop {
        Timer::after_millis(10).await;
        let mut bus_unlock = bus.lock().await; //waits for canbus peri to be free
        let try_read = bus_unlock.try_read(); //try read
        if let  Err(TryReadError::Empty) = try_read{ //if read fn returs empty:: log or nothing
            info!("no messages left");
        }else{
            info!("recived: {}", try_read.unwrap().frame.data()); //if not empty do something with data
        } 
    }
}

#[embassy_executor::task]
async fn sensor_task(mut adc4: Adc<'static, ADC4>, 
                    mut adc3: Adc<'static, ADC3>,  
                    mut ain1: AnyAdcChannel<ADC4>, 
                    mut ain2: AnyAdcChannel<ADC3>) 
{
    Timer::after_millis(50).await; 
    loop {
        Timer::after_millis(100).await;
        let ain1_reading = adc4.read(&mut ain1).await;// Read the ADC value from the specified pin
        //info!("AIN1 reading: {}", ain1_reading);
        let ain2_reading = adc3.read(&mut ain2).await;
        //info!("AIN2 reading: {}", ain2_reading);
        CAN_MESSAGE.store(ain2_reading as u32, Ordering::Relaxed);
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
    car_bus.modify_filters().enable_bank(0, can::Fifo::Fifo0, can::filter::Mask32::accept_all()); //set can filter
    car_bus.enable().await;                                                                       //enable can
    static CAN_BUS_CELL: StaticCell<can_bus_mut_type> = StaticCell::new();//static cell lets us initialize at runtime into memory reserved at compile time
    let car_bus_mutex = CAN_BUS_CELL.init(Mutex::new(car_bus)); //static cell used to init a new mutex of canbus peri

    ////////////////////////////Configure ADC peripherals
    let adc3 = Adc::new(p.ADC3, Adc3Irqs);
    let adc4 = Adc::new(p.ADC4, Adc4Irqs);

    // Spawn tasks
    spawner.spawn(can_write_task(car_bus_mutex)).unwrap();
    spawner.spawn(can_read_task(car_bus_mutex)).unwrap();
    spawner.spawn(sensor_task(adc4, adc3, p.PE8.degrade_adc(), p.PE9.degrade_adc())).unwrap();

    // Main loop (can be used for other tasks)
    loop {
        Timer::after_millis(100).await;
    }
}

pub fn u32tou8array(data: &u32) -> [u8; 4] {
    let mut res = [0; 4];
    res[..4].copy_from_slice(&data.to_le_bytes()); //copies the memory representation in slices, little endian byte order
    res //returns result
}