from __future__ import division

from autoee import Net
from autoee import kicad, bom, easypart, landpattern, model, util, harnesses, netlist_graph2
from autoee.units import INCH, MIL
from autoee.components.capacitor import capacitor as _capacitor
from autoee.components.inductor import inductor as _inductor
from autoee.components.resistor import resistor as _resistor

from autoee_components.nvs_technologies_ag import NV08C_CSM
from autoee_components.skytraq import S1315F8_RAW
from autoee_components.skyworks import AAT3221
from autoee_components._4ucon import _11029
from autoee_components.molex import _0732511150
from autoee_components.copal_electronics.CL_SB import CL_SB_22A_12
from autoee_components.murata_electronics import BLM15G
from autoee_components.laird_technologies import BMI_S_203
from autoee_components.jst import PH
from autoee_components.amphenol import _101_00660
from autoee_components.bosch_sensortec import BNO055
from autoee_components.measurement_specialties import MS5611_01BA03
from autoee_components.header import make_header
from autoee_components.stmicroelectronics import STM32F103
from autoee_components.fci import _10118194_0001LF
from autoee_components.toshiba import SSM3K15AFS
from autoee_components.silicon_labs import Si501
from autoee_components.hirose import U_FL
from autoee_components.lite_on import LTST_C19HE1WT
from autoee_components.toshiba import CUS520

microstrip_width = 0.01198*INCH

inductor = lambda *args, **kwargs: _inductor(*args, packages=frozenset({'0402 '}), **kwargs)
resistor = lambda *args, **kwargs: _resistor(*args, packages=frozenset({'0402 '}), **kwargs)
capacitor = lambda *args, **kwargs: _capacitor(*args, packages=frozenset({'0402 '}), **kwargs) # will create a problem for power filtering...

@util.listify
def main():
    gnd = Net('gnd')
    
    # BATTERY
    vbat = Net('vbat'); VBAT_MAX = 4.2
    yield PH.S2B_PH_SM4_TB(['P', 'N'])('P3',
        P=vbat,
        N=gnd,
        MECHANICAL=gnd,
    )
    
    # REGULATOR & POWER SWITCH
    vcc3_3_enable = Net('vcc3_3_enable')
    yield resistor(1e6)('R1', A=vbat, B=vcc3_3_enable)
    yield capacitor(1e-6)('C2', A=vcc3_3_enable, B=gnd)
    vcc3_3_enable_pulldown = Net('vcc3_3_enable_pulldown')
    yield resistor(10e3)('R7', A=vcc3_3_enable_pulldown, B=gnd)
    yield SSM3K15AFS.SSM3K15AFS_LF('Q1', D=vcc3_3_enable, G=vcc3_3_enable_pulldown, S=gnd)
    
    vcc3_3 = Net('v3.3')
    yield AAT3221.linear('REG_', VBAT_MAX, 3.3, vbat, gnd, vcc3_3, enable=vcc3_3_enable)
    
    # UART CONNECTOR
    port_cts = Net('port_cts')
    port_vcc = Net('port_vcc')
    port_txd = Net('port_txd')
    port_rxd = Net('port_rxd')
    port_rts = Net('port_rts')
    yield CUS520.CUS520_H3F('D1', A=vcc3_3, C=port_vcc)
    connector = _11029.make_11029('gnd cts vcc txd rxd rts'.split(' '))
    yield connector('P1',
        gnd=gnd,
        cts=port_cts,
        vcc=port_vcc,
        txd=port_txd,
        rxd=port_rxd,
        rts=port_rts,
    )
    
    # GPS RECEIVER & ANTENNA CONNECTORS
    gps_tx = Net('gps_tx')
    gps_rx = Net('gps_rx')
    
    ant1 = Net('ant1')
    yield _0732511150._0732511150(center_pad_width=microstrip_width*1.5)('P2',
        CENTER=ant1,
        SHIELD=gnd,
    )
    yield U_FL.U_FL_R_SMT_01_('P8',
        SIG=ant1,
        GND=gnd,
    )
    vin_a = Net('vin_a')
    yield BLM15G.BLM15GG471SN1D('FB1', A=vcc3_3, B=vin_a)
    yield capacitor(22e-12)('C1', A=vin_a, B=gnd)
    yield NV08C_CSM.NV08C_CSM('U1',
        GND=gnd,
        
        VIN_A=vin_a,
        VIN_D=vcc3_3,
        VBAT=vcc3_3,
        VCCIO=vcc3_3,
        
        antGND=gnd,
        RF=ant1,
        
        TX2=gps_tx,
        RX2=gps_rx,
    )
    
    yield S1315F8_RAW.S1315F8_RAW('U2',
        GND=gnd,
        AGND=gnd,
        VBAT=vin_a,
        VCC33=vin_a,
        RFIN=ant1,
        TXD0=gps_tx,
        RXD0=gps_rx,
    )
    
    
    # RF SHIELD
    yield BMI_S_203.BMI_S_203('SH', GND=gnd)
    
    # STATUS LED
    # two color led for status
    #   slow blinking red - battery dead
    #   fast blinking red - SD card error
    #   slow blinking green - waiting for acquisition
    #   fast blinking green - acquired, recording
    status_led_anode = Net('status_led_anode')
    status_led_red_cathode = Net('status_led_red_cathode')
    status_led_green_cathode = Net('status_led_green_cathode')
    status_led_blue_cathode = Net('status_led_blue_cathode')
    yield resistor(50)('R8', A=vcc3_3, B=status_led_anode)
    yield LTST_C19HE1WT.LTST_C19HE1WT('L1',
        A=status_led_anode,
        CR=status_led_red_cathode,
        CG=status_led_green_cathode,
        CB=status_led_blue_cathode,
    )
    
    # AHRS
    ahrs_i2c = harnesses.I2CBus.new('ahrs_i2c_')
    ahrs_int = Net('ahrs_int')
    yield resistor(5.6e3)('R3', A=vcc3_3, B=ahrs_i2c.SCL)
    yield resistor(5.6e3)('R4', A=vcc3_3, B=ahrs_i2c.SDA)
    U3 = yield BNO055.BNO055('U3',
        GND=gnd,
        VDD=vcc3_3,
        nBOOT_LOAD_PIN=vcc3_3,
        PS1=gnd,
        PS0=gnd,
        PIN10=gnd,
        #nRESET
        INT=ahrs_int,
        PIN15=gnd,
        PIN16=gnd,
        COM3=gnd, # I2C_ADDR_SEL
        COM2=gnd,
        COM1=ahrs_i2c.SCL,
        COM0=ahrs_i2c.SDA,
        GNDIO=gnd,
        VDDIO=vcc3_3,
    )
    yield capacitor(100e-9)('U3C1', A=U3.pin.VDD, B=gnd)
    yield capacitor(100e-9)('U3C2', A=U3.pin.CAP, B=gnd)
    yield capacitor(120e-9)('U3C3', A=U3.pin.VDDIO, B=gnd)
    yield capacitor(6.8e-9)('U3C4', A=U3.pin.VDDIO, B=gnd)
    yield resistor(10e3)('U3R1', A=U3.pin.VDDIO, B=U3.pin.nRESET)
    
    # ALTIMETER
    baro_spi = harnesses.SPIBus.new('baro_spi_')
    baro_spi_nCS = Net('baro_spi_nCS')
    yield capacitor(100e-9)('U4C', A=vcc3_3, B=gnd)
    yield MS5611_01BA03.MS5611_01BA03('U4',
        VDD=vcc3_3,
        PS=gnd, # use SPI
        GND=gnd,
        CSB=baro_spi_nCS,
        SDO=baro_spi.MISO,
        SDI=baro_spi.MOSI,
        SCLK=baro_spi.SCLK,
    )
    
    # SD CARD
    sd_spi = harnesses.SPIBus.new('sd_spi_')
    sd_spi_nCS = Net('sd_spi_nCS')
    yield _101_00660._101_00660_68_6('P4',
        G=gnd,
        VSS=gnd,
        VDD=vcc3_3,
        
        CS=sd_spi_nCS,
        DI=sd_spi.MOSI,
        CLK=sd_spi.SCLK,
        DO=sd_spi.MISO,
    )
    
    # OSCILLATOR
    XTALIN = Net('XTALIN')
    yield capacitor(0.1e-6)('U6C1', A=vcc3_3, B=gnd)
    yield Si501._501ABA8M00000DAF('U6',
        #OE
        GND=gnd,
        CLK=XTALIN,
        VDD=vcc3_3,
    )
    
    # USB port
    usb = harnesses.USB.new('usb_')
    usb_host_sense = Net('usb_host_sense')
    usb_pullup = Net('usb_pullup')
    yield resistor(1.5e3)('R6', A=usb_pullup, B=usb.Dp)
    yield _10118194_0001LF._10118194_0001LF('P7',
        VCC=usb_host_sense,
        Dm=usb.Dm,
        Dp=usb.Dp,
        #ID floats to designate slave
        GND=gnd,
        SHIELD=Net('usb_shield'),
    )
    
    # MICROCONTROLLER & DEBUG PORT
    uc_SWCLK = Net('uc_SWCLK')
    uc_SWDIO = Net('uc_SWDIO')
    uc_NRST = Net('uc_NRST')
    yield resistor(10e3)('R5', A=vcc3_3, B=uc_NRST)
    uc_SWO = Net('uc_SWO')
    yield make_header('VREF SWCLK GND SWDIO NRST SWO'.split(' '))('P5',
        VREF=vcc3_3,
        SWCLK=uc_SWCLK,
        GND=gnd,
        SWDIO=uc_SWDIO,
        NRST=uc_NRST,
        SWO=uc_SWO,
    )
    
    for i in xrange(4):
        yield capacitor(0.1e-6)('U5C%i'%i, A=vcc3_3, B=gnd)
    yield capacitor(4.7e-6)('U5C4', A=vcc3_3, B=gnd) # connect to VDD_3 (pin 48)
    yield capacitor(10e-9)('U5C6', A=vcc3_3, B=gnd) # connect to VDDA (pin 9)
    yield capacitor(1e-6)('U5C7', A=vcc3_3, B=gnd) # connect to VDDA (pin 9)
    yield STM32F103.STM32F103CBT7('U5',
        VSS=gnd, VSSA=gnd,
        VDD=vcc3_3, VDDA=vcc3_3, VBAT=vcc3_3,
        
        PD0=XTALIN, # OSC_IN
        NRST=uc_NRST,
        
        #PA4=vbat_divided, # ADC12_IN4
        
        PA2=port_rxd, # USART2_TX
        PA3=port_txd, # USART2_RX
        PA0=port_rts, # USART2_CTS
        PA1=port_cts, # USART2_RTS

        
        PA15=sd_spi_nCS, # SPI1_NSS
        PB3=sd_spi.SCLK, # SPI1_SCK
        PB4=sd_spi.MISO, # SPI1_MISO
        PB5=sd_spi.MOSI, # SPI1_MOSI
        
        PB2=ahrs_int,
        PB10=ahrs_i2c.SCL, # I2C2_SCL
        PB11=ahrs_i2c.SDA, # I2C2_SDA
        
        PB12=baro_spi_nCS, # SPI2_NSS
        PB13=baro_spi.SCLK, # SPI2_SCK
        PB14=baro_spi.MISO, # SPI2_MISO
        PB15=baro_spi.MOSI, # SPI2_MOSI
        
        PA9 =gps_rx, # USART1_TX
        PA10=gps_tx, # USART1_RX
        
        PA8=usb_host_sense, # need to be 5V tolerant!
        PA11=usb.Dm, # USBDM
        PA12=usb.Dp, # USBDP
        PB6=usb_pullup,
        
        #PB5=ahrs_int,
        #PB6=ahrs_i2c.SCL, # I2C1_SCL
        #PB7=ahrs_i2c.SDA, # I2C1_SDA
        
        PA13=uc_SWDIO, # JTMS/SWDIO
        PA14=uc_SWCLK, # JTCK/SWCLK
        #PB3=uc_SWO, # TRACESWO
        #PB4 # JNTRST
        
        PB1=vcc3_3_enable_pulldown,
        
        BOOT0=gnd, # main flash memory
        
        PA6=status_led_red_cathode, # TIM3_CH1
        PA7=status_led_green_cathode, # TIM3_CH2
        PB0=status_led_blue_cathode, # TIM3_CH3
    )

desc = main()
kicad.generate(desc, 'kicad')
bom.generate(desc, 'bom', quantity=1, add_spares=True)
#netlist_graph2.generate(desc, 'netlist.svg')
