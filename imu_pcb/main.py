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

microstrip_width = 0.01198*INCH

inductor = lambda *args, **kwargs: _inductor(*args, packages=frozenset({'0402 '}), **kwargs)
resistor = lambda *args, **kwargs: _resistor(*args, packages=frozenset({'0402 '}), **kwargs)
capacitor = lambda *args, **kwargs: _capacitor(*args, packages=frozenset({'0402 '}), **kwargs) # will create a problem for power filtering...

@util.listify
def main():
    gnd = Net('gnd')
    
    vcc3_3 = Net('vcc3_3')
    sensor_spi = harnesses.SPIBus.new('sensor_spi_')
    imu_spi_nCS = Net('imu_spi_nCS')
    
    # IMU
    imu_INT = Net('imu_INT')
    yield capacitor(10e-9)('U3C3', A=vcc3_3, B=gnd) # near VDDIO
    yield capacitor(0.1e-6)('U3C2', A=vcc3_3, B=gnd) # near VDD
    REGOUT = Net('REGOUT')
    yield capacitor(0.1e-6)('U3C1', A=REGOUT, B=gnd)
    yield MPU_9250.MPU_9250('U3',
        RESV_1=vcc3_3,
        VDDIO=vcc3_3,
        AD0_SDO=sensor_spi.MISO,
        REGOUT=REGOUT,
        #FSYNC=imu_FSYNC,
        INT=imu_INT,
        VDD=vcc3_3,
        GND=gnd,
        RESV_20=gnd,
        nCS=imu_spi_nCS,
        SCL_SCLK=sensor_spi.SCLK,
        SDA_SDI=sensor_spi.MOSI,
    )
    
    # EXTERNAL IMUs
    yield SH.SM06B_SRSS_TB('VCC GND SCLK MOSI MISO nCS'.split(' '))('P9',
        VCC=vcc3_3,
        GND=gnd,
        SCLK=sensor_spi.SCLK,
        MOSI=sensor_spi.MOSI,
        MISO=sensor_spi.MISO,
        nCS=imu_spi_nCS,
        MECHANICAL=gnd,
    )

desc = main()
kicad.generate(desc, 'kicad')
bom.generate(desc, 'bom', quantity=6, add_spares=True)
#netlist_graph2.generate(desc, 'netlist.svg')
