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
from autoee_components.molex import _0732511150
from autoee_components.copal_electronics.CL_SB import CL_SB_22A_12
from autoee_components.murata_electronics import BLM15G
from autoee_components.laird_technologies import BMI_S_202
from autoee_components.jst import PH, SH
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
from autoee_components.taoglas import GP_1575_25_4_A_02
from autoee_components.invensense import MPU_9250
from autoee_components.microchip import MCP73832
from autoee_components.on_semiconductor import NCP702
from autoee_components.epson import SG_210STF

microstrip_width = 0.01198*INCH

inductor = lambda *args, **kwargs: _inductor(*args, packages=frozenset({'0402 '}), **kwargs)
resistor = lambda *args, **kwargs: _resistor(*args, packages=frozenset({'0402 '}), **kwargs)
capacitor = lambda *args, **kwargs: _capacitor(*args, packages=frozenset({'0402 '}), **kwargs) # will create a problem for power filtering...

@util.listify
def main():
    gnd = Net('gnd')
    
    # BATTERY
    vbat = Net('vbat'); VBAT_MAX = 4.2
    
    # REGULATOR & POWER SWITCH
    vcc3_3_enable = Net('vcc3_3_enable')
    vcc3_3_enable_real = Net('vcc3_3_enable_real')
    vcc3_3_enable_uc = Net('vcc3_3_enable_uc')
    vcc3_3 = Net('v3.3')
    
    yield capacitor(2.2e-6)('C1', A=vbat, B=vcc3_3_enable)
    yield resistor(90e3)('R2', A=vcc3_3_enable_uc, B=vcc3_3_enable)
    yield resistor(100e3)('R3', A=vcc3_3_enable, B=vcc3_3_enable_real)
    yield resistor(2000e3)('R4', A=vcc3_3_enable, B=vcc3_3)
    
    yield capacitor(1e-6)('U1C1', A=vbat, B=gnd) # PCB: put close to U1
    yield NCP702.by_voltage[3.3]('U1',
        IN=vbat,
        GND=gnd,
        EN=vcc3_3_enable_real,
        OUT=vcc3_3,
        NC=gnd, # thermal
    )
    yield capacitor(1e-6)('U1C2', A=vcc3_3, B=gnd) # PCB: put close to U1
    
    # GPS RECEIVER & ANTENNA CONNECTORS
    gps_tx = Net('gps_tx')
    gps_rx = Net('gps_rx')
    
    ant1 = Net('ant1')
    yield U_FL.U_FL_R_SMT_01_('P8',
        SIG=ant1,
        GND=gnd,
    )
    yield S1315F8_RAW.S1315F8_RAW('U2',
        GND=gnd,
        AGND=gnd,
        VBAT=vcc3_3,
        VCC33=vcc3_3,
        RFIN=ant1,
        TXD0=gps_tx,
        RXD0=gps_rx,
    )
    
    
    # RF SHIELD
    #yield BMI_S_202.BMI_S_202('SH', GND=gnd)
    
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
    
    # SD CARD
    sd_spi = harnesses.SPIBus.new('sd_spi_')
    sd_spi_nCS = Net('sd_spi_nCS')
    P4 = yield _101_00660._101_00660_68_6('P4',
        G=gnd,
        VSS=gnd,
        VDD=vcc3_3,
        
        CS=sd_spi_nCS,
        DI=sd_spi.MOSI,
        CLK=sd_spi.SCLK,
        DO=sd_spi.MISO,
    )
    yield capacitor(10e-6)('P4C1', A=P4.pin.VDD, B=P4.pin.VSS)
    
    # OSCILLATOR
    XTALIN = Net('XTALIN')
    yield capacitor(0.1e-6)('U6C1', A=vcc3_3, B=gnd)
    #yield Si501._501ABA8M00000DAF('U6',
    #    #OE
    #    GND=gnd,
    #    CLK=XTALIN,
    #    VDD=vcc3_3,
    #)
    yield SG_210STF.SG_210STF_8_0000ML('U6',
        #OE
        GND=gnd,
        OUT=XTALIN,
        VCC=vcc3_3,
    )
    
    # DEBUG/BATTERY/UART PORT
    uc_SWCLK = Net('uc_SWCLK')
    uc_SWDIO = Net('uc_SWDIO')
    uc_NRST = Net('uc_NRST')
    yield resistor(10e3)('R5', A=vcc3_3, B=uc_NRST)
    serial_txd = Net('serial_txd')
    serial_rxd = Net('serial_rxd')
    yield SH.SM07B_SRSS_TB('GND SWDIO SWCLK TX RX NRST VBAT'.split(' '))('P5',
        VBAT=vbat,
        GND=gnd,
        SWCLK=uc_SWCLK,
        SWDIO=uc_SWDIO,
        NRST=uc_NRST,
        TX=serial_txd,
        RX=serial_rxd,
        MECHANICAL=gnd,
    )
    
    for i in xrange(3):
        yield capacitor(0.1e-6)('U5C%i'%i, A=vcc3_3, B=gnd) # PCB: pins 1, 19, and 27
    yield capacitor(4.7e-6)('U5C4', A=vcc3_3, B=gnd) # PCB: connect to VDD_3 (pin 1)
    yield capacitor(10e-9)('U5C6', A=vcc3_3, B=gnd) # PCB: connect to VDDA (pin 6)
    yield capacitor(1e-6)('U5C7', A=vcc3_3, B=gnd) # PCB: connect to VDDA (pin 6)
    yield STM32F103.STM32F103TBU6('U5',
        VSS=gnd, VSSA=gnd,
        VDD=vcc3_3, VDDA=vcc3_3, #VBAT=vcc3_3,
        
        PD0=XTALIN, # OSC_IN
        NRST=uc_NRST,
        
        PA2=gps_rx, # USART2_TX
        PA3=gps_tx, # USART2_RX
        
        PB6=serial_txd, # USART1_TX (5V tolerant)
        PB7=serial_rxd, # USART1_RX
        #PA9 =serial_txd, # USART1_TX (5V tolerant)
        #PA10=serial_rxd, # USART1_RX
        
        PA15=sd_spi_nCS,  # SPI1_NSS
        PB3 =sd_spi.SCLK, # SPI1_SCK
        PB4 =sd_spi.MISO, # SPI1_MISO
        PB5 =sd_spi.MOSI, # SPI1_MOSI
        #PA4=sd_spi_nCS,  # SPI1_NSS
        #PA5=sd_spi.SCLK, # SPI1_SCK
        #PA6=sd_spi.MISO, # SPI1_MISO
        #PA7=sd_spi.MOSI, # SPI1_MOSI
        
        PA13=uc_SWDIO, # JTMS/SWDIO
        PA14=uc_SWCLK, # JTCK/SWCLK
        #PB3=uc_SWO, # TRACESWO
        #PB4 # JNTRST
        
        PB1=vcc3_3_enable_uc, # must not be 5V tolerant (injected current limits)
        
        BOOT0=gnd, # main flash memory
        
        PA6=status_led_red_cathode, # TIM3_CH1
        PA7=status_led_green_cathode, # TIM3_CH2
        PB0=status_led_blue_cathode, # TIM3_CH3
    )

desc = main()
kicad.generate(desc, 'kicad')
bom.generate(desc, 'bom', quantity=3, add_spares=True)
#netlist_graph2.generate(desc, 'netlist.svg')
