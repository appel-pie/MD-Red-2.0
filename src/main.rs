    #![no_std]
    #![no_main]


use cortex_m::delay;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::can::frame::Envelope;
use embassy_stm32::can::util::NominalBitTiming;
use embassy_stm32::{adc, init, peripherals::*, peripherals, bind_interrupts, can, Config, rcc, gpio};
use embassy_stm32::adc::{AdcChannel, Adc, AnyAdcChannel};
use embassy_stm32::can::Can;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::can::{Frame, StandardId};
use embassy_stm32::can::enums::TryReadError;

use embassy_stm32::peripherals::ADC4;
use embassy_stm32::time::mhz;
use embassy_time::Timer;
use core::num::{NonZeroU16, NonZeroU8};
use core::sync::atomic::{AtomicU32, Ordering};
use {defmt_rtt as _, panic_probe as _,}; 


//////////////mutex includes////////////
// use core::cell::RefCell;
// use embassy_sync::blocking_mutex::Mutex;
// use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;

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
/////////////////can interupt create
//static TEST: can::bxcan::Interrupt = can::bxcan::Interrupt::Fifo0Full;

//////////////////////////////////static variables
static CAN_MESSAGE: AtomicU32 = AtomicU32::new(0);

//type AinType = Mutex<ThreadModeRawMutex, Option<adc::AdcPin<'static, adc::AnyAdcChannel>>>;
//static AIN1: AinType = Mutex::new(None);

// type PinType = Mutex<ThreadModeRawMutex, Option<Output<'static, AnyPin>>>;
// static start_button: PinType = Mutex::new(None);
// static throttle_sens_1: PinType = Mutex::new(None);



#[embassy_executor::task]
async fn can_task(mut bus: Can<'static>) {
    info!("can task begin");
    let id = StandardId::new(100).unwrap();
    loop {  
        Timer::after_millis(10).await;
        let can_frame: Frame = Frame::new_data(id, &u32tou8array(&CAN_MESSAGE.load(Ordering::Relaxed))).unwrap();
        bus.write(&can_frame).await;

        let try_read = bus.try_read();
        if let  Err(TryReadError::Empty) = try_read{ 
            info!("no messages left");
        }else{
            info!("recived: {}", try_read.unwrap().frame.data());
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
        // Read the ADC value from the specified pin
        let ain1_reading = adc4.read(& mut ain1).await;
        //info!("AIN1 reading: {}", ain1_reading);
        let ain2_reading = adc3.read(&mut ain2).await;
        //info!("AIN2 reading: {}", ain2_reading);
        CAN_MESSAGE.store(ain2_reading as u32, Ordering::Relaxed);
    }
}




#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("main entry");
    // Initialize and create handle for devicer peripherals
    let mut config = Config::default();
    {
        config.rcc.hse = Some(rcc::Hse {freq: mhz(32), mode: rcc::HseMode::Oscillator});

        config.rcc.hsi = false; //disable internal clock 
        //pll at 64mHz (idk why but it dosent like div2 mul4)
        config.rcc.pll = Some(rcc::Pll {src: rcc::PllSource::HSE, prediv: rcc::PllPreDiv::DIV8, mul: rcc::PllMul::MUL16});
        //sysclock gets high speed external clock, 32mHz
        config.rcc.sys = rcc::Sysclk::HSE;
        //hclk gets sysclk div4: 8mHz
        config.rcc.ahb_pre =  rcc::AHBPrescaler::DIV4; 
        //ADC Clocks pll div4: 16mHz    
        config.rcc.adc =  rcc::AdcClockSource::Pll(rcc::AdcPllPrescaler::DIV4); 
        config.rcc.adc34 =  rcc::AdcClockSource::Pll(rcc::AdcPllPrescaler::DIV4);
    }
    let p = init(config);
    info!("configed");

    /////////////////////////////Configure Can Peripheral
    let mut car_bus: Can<'static> = Can::new(p.CAN, p.PD0, p.PD1, CanIrqs);
    let canconfig = car_bus.modify_config();
    canconfig.set_bit_timing(NominalBitTiming{ ////////////// Note: bit timings need manual settings, not sure why
        prescaler: NonZeroU16::new(1).unwrap(),
        seg1: NonZeroU8::new(5).unwrap(),
        seg2: NonZeroU8::new(2).unwrap(),
        sync_jump_width: NonZeroU8::new(8).unwrap(),
    });
     //http://www.bittiming.can-wiki.info/#bxCAN
    // BitRate accuracy    Pre-scaler  Number of time quanta   seg 1   Seg 2	Sample Point    Register
    // 1000	0.0000	       1	       8	                   5	   2	    75.0	        0x00140000
    car_bus.modify_filters().enable_bank(0, can::Fifo::Fifo0, can::filter::Mask32::accept_all());
    

    car_bus.enable().await;
    //car_bus.enable_interrupt(TEST); //Todo: remove or implement can interrupt

    ////////////////////////////Configure ADC peripherals
    let adc3 = Adc::new(p.ADC3, Adc3Irqs);
    let adc4 = Adc::new(p.ADC4, Adc4Irqs);

    let mut on_button: ExtiInput<'_> = ExtiInput::new(p.PE2, p.EXTI2, gpio::Pull::None);
    
    // Spawn tasks
    spawner.spawn(can_task(car_bus)).unwrap();
    spawner.spawn(sensor_task(adc4, adc3, 
                p.PE8.degrade_adc(), 
                p.PE9.degrade_adc())).unwrap();

    // loop {
    //     //info!("main loop begin");
    //     //Check if button got pressed
    //     // on_button.wait_for_rising_edge().await;
    //     // info!("button rising edge detected");
    //      Timer::after_millis(100).await;
    //     // if on_button.is_high() 
    //     // {
    //     // info!("State transition");
    //     // CAN_MESSAGE.fetch_add(1, Ordering::Relaxed);
    //     // }

    //     let result1 = adc3.read(&mut p.PE9).await as u32;
    //     let result2 = adc3.read(&mut p.PE10).await as u32;
    //     info!("result1: {}, result2: {}", result1, result2);

    //     CAN_MESSAGE.store(result1, Ordering::Relaxed);
    //     //info!("adc's read and sent");
    // }
}

pub fn u32tou8array(data: &u32) -> [u8; 4] {
    let mut res = [0; 4];
    res[..4].copy_from_slice(&data.to_le_bytes()); //copies the memory representatin in slices, little endian byte order
    res //returns result
}

// pub fn arrayu8tou32(array: &[u8; 4]) -> u32 {
//     let mut res: u32 = 0;
// }

// pub struct Sensor<'a, T>{
//     sensor_limits: SensorCharacteristics,
//     pin: &'a mut Peripheral,
// }

// pub struct SensorCharacteristics{
//     upper_range: u32,
//     lower_range: u32, 
// }