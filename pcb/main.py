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
from autoee_components import mounting_hole
from autoee_components.copal_electronics.CL_SB import CL_SB_22A_12
from autoee_components.murata_electronics import BLM15G
from autoee_components.laird_technologies import BMI_S_203
from autoee_components.jst import PH
from autoee_components.amphenol import _101_00660
from autoee_components.bosch_sensortec import BNO055
from autoee_components.measurement_specialties import MS5611_01BA03
from autoee_components.header import make_header
from autoee_components.stmicroelectronics import STM32F103

microstrip_width = 0.01198*INCH

inductor = lambda *args, **kwargs: _inductor(*args, packages=frozenset({'0402 '}), **kwargs)
resistor = lambda *args, **kwargs: _resistor(*args, packages=frozenset({'0402 '}), **kwargs)
capacitor = lambda *args, **kwargs: _capacitor(*args, packages=frozenset({'0402 '}), **kwargs) # will create a problem for power filtering...

# two color led for status
#   slow blinking red - battery dead
#   fast blinking red - SD card error
#   slow blinking green - waiting for acquisition
#   fast blinking green - acquired, recording


connector = _11029.make_11029('gnd cts vcc5 txd rxd rts'.split(' '))

@util.listify
def main():
    gnd = Net('gnd')
    
    for i in xrange(2):
        yield mounting_hole.mounting_hole('M' + str(i+1))
    
    # BATTERY
    
    vbat = Net('vbat'); VBAT_MAX = 4.2
    yield PH.S2B_PH_SM4_TB(['P', 'N'])('P3',
        P=vbat,
        N=gnd,
        MECHANICAL=gnd,
    )
    
    # REGULATOR
    
    vcc3_3_enable = Net('vcc3_3_enable')
    yield resistor(10e3)('R1', A=vbat, B=vcc3_3_enable)
    yield capacitor(1e-6)('R2', A=vcc3_3_enable, B=gnd)
    vcc3_3_enable_pulldown = Net('vcc3_3_enable_pulldown')
    #yield mosfet('Q1', D=vcc3_3_enable, G=vcc3_3_enable_pulldown, S=gnd) XXX
    
    vcc3_3 = Net('v3.3')
    yield AAT3221.linear('REG_', VBAT_MAX, 3.3, vbat, gnd, vcc3_3, enable=vcc3_3_enable)
    
    # UART CONNECTOR
    
    port_cts = Net('port_cts')
    port_txd = Net('port_txd')
    port_rxd = Net('port_rxd')
    port_rts = Net('port_rts')
    yield connector('P1',
        gnd=gnd,
        cts=port_cts,
        txd=port_txd,
        rxd=port_rxd,
        rts=port_rts,
    )
    
    # GPS RECEIVER
    
    gps_tx = Net('gps_tx')
    gps_rx = Net('gps_rx')
    
    # XXX 5V antenna power? bias tee?
    
    ant1 = Net('ant1')
    yield _0732511150._0732511150(center_pad_width=microstrip_width)('P2',
        CENTER=ant1,
        SHIELD=gnd,
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
    
    ant2 = Net('ant2')
    yield _0732511150._0732511150(center_pad_width=microstrip_width)('P6',
        CENTER=ant2,
        SHIELD=gnd,
    )
    yield S1315F8_RAW.S1315F8_RAW('U2',
        GND=gnd,
        AGND=gnd,
        VBAT=vin_a,
        VCC33=vin_a,
        RFIN=ant2,
        TXD0=gps_tx,
        RXD0=gps_rx,
    )
    
    yield BMI_S_203.BMI_S_203('SH', GND=gnd)
    
    # STATUS LED
    
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
    
    # XXX
    
    # MICROCONTROLLER
    
    uc_SWCLK = Net('uc_SWCLK')
    uc_SWDIO = Net('uc_SWDIO')
    uc_NRST = Net('uc_NRST') # XXX pullup?
    uc_SWO = Net('uc_SWO')
    yield make_header('VREF SWCLK GND SWDIO NRST SWO'.split(' '))('P5',
        VREF=vcc3_3,
        SWCLK=uc_SWCLK,
        GND=gnd,
        SWDIO=uc_SWDIO,
        NRST=uc_NRST,
        SWO=uc_SWO,
    )
    
    # XXX decoupling
    yield STM32F103.STM32F103CBT7('U5',
        VSS=gnd, VSSA=gnd,
        VDD=vcc3_3, VDDA=vcc3_3,
        
        #PD0 # OSC_IN # XXX
        NRST=uc_NRST,
        
        PA0=vbat, # ADC12_IN0 XXX resistor divider
        
        PA2=gps_rx, # USART2_TX
        PA3=gps_tx, # USART2_RX
        
        PA4=baro_spi_nCS, # SPI1_NSS
        PA5=baro_spi.SCLK, # SPI1_SCK
        PA6=baro_spi.MISO, # SPI1_MISO
        PA7=baro_spi.MOSI, # SPI1_MOSI
        
        PB2=ahrs_int,
        PB10=ahrs_i2c.SCL, # I2C2_SCL
        PB11=ahrs_i2c.SDA, # I2C2_SDA
        
        PB12=sd_spi_nCS, # SPI2_NSS
        PB13=sd_spi.SCLK, # SPI2_SCK
        PB14=sd_spi.MISO, # SPI2_MISO
        PB15=sd_spi.MOSI, # SPI2_MOSI
        
        PA9 =port_rxd, # USART1_TX
        PA10=port_txd, # USART1_RX
        PA11=port_rts, # USART1_CTS
        PA12=port_cts, # USART1_RTS
        
        #PB5=ahrs_int,
        #PB6=ahrs_i2c.SCL, # I2C1_SCL
        #PB7=ahrs_i2c.SDA, # I2C1_SDA
        
        PA13=uc_SWDIO, # JTMS/SWDIO
        PA14=uc_SWCLK, # JTCK/SWCLK
        PB3=uc_SWO, # TRACESWO
        #PB4 # JNTRST XXX
        
        BOOT0=gnd, # main flash memory
    )

desc = main()
kicad.generate(desc, 'kicad')
bom.generate(desc, 'bom', quantity=1, add_spares=False)
#netlist_graph2.generate(desc, 'netlist.svg')
