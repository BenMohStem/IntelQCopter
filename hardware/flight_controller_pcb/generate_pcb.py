"""Generate the provisional, unrouted Pico 2 flight-controller carrier PCB."""
from pathlib import Path
import sys

import pcbnew


PROJECT_DIR = Path(__file__).resolve().parent
KIROOT = Path(sys.executable).resolve().parent.parent
FOOTPRINT_ROOT = KIROOT / "share" / "kicad" / "footprints"
OUTPUT = PROJECT_DIR / "qcopter_fc_carrier.kicad_pcb"


def mm(value: float) -> int:
    return pcbnew.FromMM(value)


board = pcbnew.BOARD()
board.SetFileName(str(OUTPUT))
board.SetCopperLayerCount(4)

nets = {}


def net(name: str):
    if name not in nets:
        item = pcbnew.NETINFO_ITEM(board, name)
        board.Add(item)
        nets[name] = item
    return nets[name]


def add_footprint(ref, value, lib, name, x, y, angle=0, pad_nets=None):
    footprint_path = FOOTPRINT_ROOT / f"{lib}.pretty"
    fp = pcbnew.FootprintLoad(str(footprint_path), name)
    if fp is None:
        raise RuntimeError(f"Footprint not found: {lib}:{name}")
    fp.SetFPIDAsString(f"{lib}:{name}")
    fp.SetReference(ref)
    fp.SetValue(value)
    fp.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
    fp.SetOrientationDegrees(angle)
    board.Add(fp)
    for pad in fp.Pads():
        assigned = (pad_nets or {}).get(str(pad.GetNumber()))
        if assigned:
            pad.SetNet(net(assigned))
    return fp


def add_edge_line(x1, y1, x2, y2):
    line = pcbnew.PCB_SHAPE(board)
    line.SetShape(pcbnew.SHAPE_T_SEGMENT)
    line.SetStart(pcbnew.VECTOR2I(mm(x1), mm(y1)))
    line.SetEnd(pcbnew.VECTOR2I(mm(x2), mm(y2)))
    line.SetLayer(pcbnew.Edge_Cuts)
    line.SetWidth(mm(0.05))
    board.Add(line)


def add_label(text_value, x, y, angle=0, size=1.0):
    text = pcbnew.PCB_TEXT(board)
    text.SetText(text_value)
    text.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
    text.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    text.SetTextThickness(mm(0.15))
    text.SetTextAngle(pcbnew.EDA_ANGLE(angle, pcbnew.DEGREES_T))
    text.SetLayer(pcbnew.F_SilkS)
    board.Add(text)


GND = "GND"
V3 = "+3V3"
V5 = "+5V_SYS"
VBAT = "VBAT_3S"
SDA = "I2C0_SDA"
SCL = "I2C0_SCL"
SCK = "SPI0_SCK"
MOSI = "SPI0_MOSI"
MISO = "SPI0_MISO"
LORA_CS = "LORA_NSS"
LORA_RST = "LORA_RESET"
LORA_DIO0 = "LORA_DIO0"
CC1_CS = "CC1101_A_CS"
CC2_CS = "CC1101_B_CS"
CC1_GDO0 = "CC1101_A_GDO0"
CC1_GDO2 = "CC1101_A_GDO2"
CC2_GDO0 = "CC1101_B_GDO0"
CC2_GDO2 = "CC1101_B_GDO2"
UART_TX = "AUX_UART_TX"
UART_RX = "AUX_UART_RX"
ESC_FL = "ESC_FL_SIG"
ESC_FR = "ESC_FR_SIG"
ESC_RR = "ESC_RR_SIG"
ESC_RL = "ESC_RL_SIG"
VBAT_ADC = "VBAT_ADC"

# Pico 2 side A is physical pins 1-20, top to bottom with USB at the top.
left_pico = {
    1: "BUZZER", 2: "ARM_SENSE", 3: GND, 4: ESC_FL, 5: ESC_FR,
    6: SDA, 7: SCL, 8: GND, 9: ESC_RR, 10: ESC_RL,
    11: UART_TX, 12: UART_RX, 13: GND, 14: LORA_DIO0,
    15: LORA_RST, 16: CC1_GDO0, 17: CC1_GDO2, 18: GND,
    19: CC2_GDO0, 20: CC2_GDO2,
}

# Pico 2 side B is physical pins 40-21, top to bottom.
right_pico = {
    1: "VBUS_USB", 2: V5, 3: GND, 4: "3V3_EN", 5: V3,
    6: "ADC_VREF", 7: GND, 8: "GPIO_SPARE_28", 9: "GPIO_SPARE_27",
    10: VBAT_ADC, 11: "RP2350_RUN", 12: "GPIO_SPARE_22", 13: GND,
    14: CC2_CS, 15: CC1_CS, 16: MOSI, 17: SCK, 18: GND,
    19: LORA_CS, 20: MISO,
}

# Board outline: 85 x 85 mm. 4-layer stack is reserved for future plane/routing.
board_width = 95.0
board_height = 85.0
for edge in (
    (0, 0, board_width, 0),
    (board_width, 0, board_width, board_height),
    (board_width, board_height, 0, board_height),
    (0, board_height, 0, 0),
):
    add_edge_line(*edge)

for ref, x, y in (
    ("H1", 27.25, 27.25), ("H2", 57.75, 27.25),
    ("H3", 27.25, 57.75), ("H4", 57.75, 57.75),
):
    add_footprint(ref, "M3 mounting hole (NPTH)", "MountingHole",
                  "MountingHole_3.2mm_M3", x, y)

socket_lib = "Connector_PinSocket_2.54mm"
socket_name = "PinSocket_1x20_P2.54mm_Vertical"
add_footprint("J1", "Pico 2 pins 1-20", socket_lib, socket_name,
              33.61, 18.37, pad_nets={str(k): v for k, v in left_pico.items()})
add_footprint("J2", "Pico 2 pins 40-21", socket_lib, socket_name,
              51.39, 18.37, pad_nets={str(k): v for k, v in right_pico.items()})

pin2 = "PinHeader_1x02_P2.54mm_Vertical"
pin3 = "PinHeader_1x03_P2.54mm_Vertical"
pin4 = "PinHeader_1x04_P2.54mm_Vertical"
pin8 = "PinHeader_1x08_P2.54mm_Vertical"
header_lib = "Connector_PinHeader_2.54mm"

# Low-current 3S power branch supplies the external buck converter only.
add_footprint("J3", "3S input (low-current branch)", header_lib, pin2,
              12, 10, 90, {"1": VBAT, "2": GND})
add_footprint("J4", "External 5 V buck module: VIN/GND/OUT", header_lib, pin3,
              36, 10, 90, {"1": VBAT, "2": GND, "3": V5})

res_lib = "Resistor_SMD"
cap_lib = "Capacitor_SMD"
add_footprint("R1", "100k battery divider", res_lib,
              "R_0805_2012Metric", 53, 9, 90, {"1": VBAT, "2": VBAT_ADC})
add_footprint("R2", "27k battery divider", res_lib,
              "R_0805_2012Metric", 59, 9, 90, {"1": VBAT_ADC, "2": GND})
add_footprint("C1", "100n ADC filter", cap_lib,
              "C_0805_2012Metric", 65, 9, 90, {"1": VBAT_ADC, "2": GND})

sensor_pads = {"1": V3, "2": GND, "3": SDA, "4": SCL}
for ref, value, y in (
    ("J5", "MPU6050 I2C", 20),
    ("J6", "BMP280 I2C", 33),
    ("J7", "HMC5883L I2C", 46),
):
    add_footprint(ref, value, header_lib, pin4, 8, y, pad_nets=sensor_pads)
add_footprint("J15", "AUX UART 3V3/GND/TX/RX", header_lib, pin4,
              8, 60, pad_nets={"1": V3, "2": GND, "3": UART_TX, "4": UART_RX})

radio_lora = {
    "1": V3, "2": GND, "3": SCK, "4": MOSI,
    "5": MISO, "6": LORA_CS, "7": LORA_RST, "8": LORA_DIO0,
}
radio_a = {
    "1": V3, "2": GND, "3": SCK, "4": MOSI,
    "5": MISO, "6": CC1_CS, "7": CC1_GDO0, "8": CC1_GDO2,
}
radio_b = {
    "1": V3, "2": GND, "3": SCK, "4": MOSI,
    "5": MISO, "6": CC2_CS, "7": CC2_GDO0, "8": CC2_GDO2,
}
add_footprint("J8", "LoRa SPI/DIO (3V3)", header_lib, pin8, 17, 78, 90, radio_lora)
add_footprint("J9", "CC1101 A SPI/GDO (3V3)", header_lib, pin8, 44, 78, 90, radio_a)
add_footprint("J10", "CC1101 B SPI/GDO (3V3)", header_lib, pin8, 71, 78, 90, radio_b)

for ref, value, y, signal in (
    ("J11", "ESC FL signal/GND", 17, ESC_FL),
    ("J12", "ESC FR signal/GND", 29, ESC_FR),
    ("J13", "ESC RR signal/GND", 41, ESC_RR),
    ("J14", "ESC RL signal/GND", 53, ESC_RL),
):
    add_footprint(ref, value, header_lib, pin2, 77, y, 90,
                  {"1": signal, "2": GND})

add_label("QCOPTER FC - DRAFT", 26, 5, size=1.2)
add_label("PICO2", 39, 73, size=1.0)
add_label("3S LOGIC ONLY", 52, 5, size=0.8)

pcbnew.SaveBoard(str(OUTPUT), board)
print(f"Wrote {OUTPUT}")
print(f"Footprints: {len(board.GetFootprints())}, nets: {board.GetNetCount()}")
