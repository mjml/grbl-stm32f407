#!/usr/bin/python3

import sys
import yaml
import re
import string


from string import Template

inputfn = "pins.yaml"
template_cfn = "grbl/gpio_map.template.c"
template_hfn = "grbl/gpio_map.template.h"
class SubstitutionData:
    vardefines = ""
    varproto = ""
    vardefn = ""
    ithandles = ""
    axes = []
    itpindefs = []

sdata = SubstitutionData()

pinexpr = re.compile("([A-Za-z])(\d{1,2})\s*(.*)")

interrupt_names = [ "reset", "feed_hold", "cycle_start", "safety_door", "probe", "alm", "limit", "home" ]

class PinDefinition:
    name,type,port,pin,mode,pull = ("", "", "", "", "", "")
    portletter, bitnumber = ("", "")
    it, input = (False, False)
    pu, pd, od, pp = ("", "", "", "")
    defnexpr = ""

    def asdict():
        return {'name': self.name, 'port': self.port, 'pin': self.pin, 'mode': self.mode, 'pull':self.pull, 'portletter':self.portletter, 'bitnumber':self.bitnumber }

    def parse (name,pinstr):
        self = PinDefinition()
        self.name = name
        self.type = "out"
        self.defnexpr = ""
        pinstr = pinstr.strip()
        match = pinexpr.match(pinstr)
        if match is None:
            raise RuntimeError("Unable to parse pin configuration string: \"{0}\"".format(pinstr))
        self.portletter = match.group(1).upper()
        self.bitnumber  = int(match.group(2))
        adjectives = match.group(3)
        self.polarity = "true" if "rev" in adjectives else "false"
        self.pu = "pu" in adjectives
        self.pd = "pd" in adjectives
        self.od = "od" in adjectives
        self.pp = "pp" in adjectives
        it = (name in interrupt_names ) or ("!" in name)
        self.input = it or name.startswith("in")
        if (self.od and self.pp):
            print("Warning: both od and pp adjective flags used in pin definition: \"{0}\"".format(pinstr))
        
        if self.it:
            self.mode = "GPIO_MODE_IT_RISING_FALLING"
        elif self.od:
            self.mode = "GPIO_MODE_OUTPUT_OD"
        elif self.pp:
            self.mode = "GPIO_MODE_OUTPUT_PP"
        elif self.input:
            self.mode = "GPIO_MODE_INPUT"
        else:
            self.mode = "GPIO_MODE_OUTPUT_PP"

        if self.pu:
            self.pull = "GPIO_PULLUP"
        elif self.pd:
            self.pull = "GPIO_PULLDOWN"
        else:
            self.pull = "GPIO_NOPULL"

        if self.name.startswith("limit"):
            self.type = "limit"
        elif self.name.startswith("home"):
            self.type = "home"
        elif "alm" in adjectives:
            self.type = "encoder_alarm"
        elif "interrupt" in adjectives:
            self.type = "interrupt"
        elif "out" in adjectives:
            self.type = "out"
        elif "in" in adjectives:
            self.type = "in"

        self.defnexpr = '{{ GPIO{0}, {1:>2}, bit({1:>2}), {2:>5}, false, false, {3}, {4}, GPIO_SPEED_FREQ_LOW, 0 }}'.format(self.portletter, self.bitnumber, self.polarity, self.mode, self.pull)
        return self


def parse_config (fn):
    global inputfile 
    inputfile = fn
    with open(fn, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as ex:
            raise ex


def get_axis (name):
    global sdata
    return sdata.axes.index(name)


def parse_axes (config, sdata):
    '''This sets up an axis enumeration and corresponding preprocessor macros'''
    title = "//// Definitions ////////////////\n"
    sdata.vardefines += title
    if not "axes" in config:
        raise RuntimeError("Couldn't find axes definitions in {0}, bailing...".format(inputfile))
    if type(config["axes"]) is not list:
        raise RuntimeError("Expected list of strings under \"axes\" label")
    for i in range(len(config["axes"])):
        sdata.axes.append(config["axes"][i])
    for i,axis in enumerate(sdata.axes):
        sdata.vardefines += "#define AXIS_{0} {1}\n".format(axis.upper(),i)


def create_system_defns (config, sdata):
    '''Creates the required reset input pin, and optional cycle_start, feed_hold, and safety_door input pins'''
    title = "//// System variables ////////////////\n"
    sdata.vardefines += "\n"
    sdata.varproto += title
    sdata.vardefn += title
    if not ("system" in config):
        raise RuntimeError("No system block found in {0}, bailing...".format(inputfile))
    sysblock = config["system"]
    if "reset" in sysblock:
        sdata.vardefines += "#define ENABLE_RESET\n"
        sdata.varproto += "extern gpio_t reset;\n"
        resetpin = PinDefinition.parse("reset",sysblock["reset"])
        resetpin.type = "system"
        sdata.itpindefs.append(resetpin)
        sdata.vardefn += "gpio_t reset = {0};\n".format(resetpin.defnexpr)
    else:
        raise RuntimeError("No reset pin found in system block.")
    
    if "cycle_start" in sysblock:
        sdata.vardefines += "#define ENABLE_CYCLE_START\n"
        sdata.varproto += "extern gpio_t cycle_start;\n"
        cspin = PinDefinition.parse("cycle_start",sysblock["cycle_start"])
        cspin.type = "system"
        sdata.itpindefs.append(cspin)
        sdata.vardefn += "gpio_t cycle_start = {0};\n".format(cspin.defnexpr)

    if "feed_hold" in sysblock:
        sdata.vardefines += "#define ENABLE_FEED_HOLD\n"
        sdata.varproto += "extern gpio_t feed_hold;\n"
        fhpin = PinDefinition.parse("feed_hold",sysblock["feed_hold"])
        fhpin.type = "system"
        sdata.itpindefs.append(fhpin)
        sdata.vardefn += "gpio_t feed_hold = {0};\n".format(fhpin.defnexpr)

    if "safety_door" in sysblock:
        sdata.vardefines += "#define ENABLE_SAFETY_DOOR\n"
        sdata.vardefines += "#define ENABLE_SAFETY_DOOR_INPUT_PIN\n"
        sdata.varproto += "extern gpio_t safety_door;\n"
        sdpin = PinDefinition.parse("safety_door",sysblock["safety_door"])
        sdpin.type = "system"
        sdata.itpindefs.append(sdpin)
        sdata.vardefn += "gpio_t safety_door = {0};\n".format(sdpin.defnexpr)
    elif "feed_hold" in sysblock:
        sdata.vardefines += "#define ENABLE_SAFETY_DOOR\n"
        sdata.varproto += "extern gpio_t safety_door;\n"
        sdata.vardefn += "gpio_t safety_door = {0};\n".format(fhpin.defnexpr)

    if "probe" in sysblock:
        sdata.vardefines += "#define ENABLE_PROBE\n"
        sdata.varproto += "extern gpio_t probe;\n"
        probe = PinDefinition.parse("probe", sysblock["probe"])
        probe.type = "system"
        sdata.itpindefs.append(probe)
        sdata.vardefn += "gpio_t probe = {0};\n".format(probe.defnexpr)

    sdata.vardefn += "\n\n"
    sdata.varproto += "\n\n"
    

def create_motor_defns (config, sdata):
    '''Configures the stepper motor pins with optional encoder alarm input'''
    title = "//// Motors ///////////\n"
    sdata.varproto += title
    sdata.vardefn += title
    encoder_input_found = False
    motors = config["motors"]
    if len(motors) > len(sdata.axes):
        raise RuntimeError("More motor definitions than defined axes.")
    blocks = [""] * len(motors)
    for i in range(len(motors)):
        m = motors[i]
        if not "axis" in m:
            raise RuntimeError("Couldn't find axis name in motor definition")
        axis = m["axis"]
        gpiodefs = []
        for s in m.keys():
            if s != "axis":
                pd = PinDefinition.parse(s, m[s])
                gpiodefs.append("  " + pd.defnexpr)
            if s == "alm":
                encoder_input_found = True
                sdata.itpindefs.append(pd)
                pd.name = "motor[AXIS_{0}].alm".format(axis.upper())

        blocks[get_axis(axis)] = "{{\n  // {0} axis\n{1}\n}}".format(axis,",\n".join(gpiodefs))
    
    if encoder_input_found:
        sdata.vardefines += "#define STEPPER_ENCODER_ALARM\n"

    sdata.varproto += "#define NUM_MOTORS {0}\n".format(len(motors))
    sdata.varproto += "extern stepper_t motors[NUM_MOTORS];\n\n"
    sdata.vardefn += "stepper_t motors[NUM_MOTORS] = {{ {0} }};".format((',\n').join(blocks))
    sdata.vardefn += "\n\n"
    

def create_limit_defns (config, sdata):
    '''Configures limit and homing pins'''
    title = "///// Limits & Homing ////////////\n"
    protos = title
    defns = title
    
    limits = config["limits"]
    if len(limits) > len(sdata.axes):
        raise RuntimeError("More limits than defined axes.")
    blocks = [""] * len(limits)
    for i in range(len(limits)):
        l = limits[i]
        if not "axis" in l:
            raise RuntimeError("Couldn't find axis in limit definition")
        if not "limit" in l:
            raise RuntimeError("Couldn't find the limit pin definition in limits item {0}".format(i))
        axis = l["axis"]
        pd = PinDefinition.parse("limit[AXIS_{0}]".format(axis.upper()),l["limit"])
        blocks[get_axis(l["axis"])] = "  // {0} axis\n  {1}".format(l["axis"],pd.defnexpr)
        sdata.itpindefs.append(pd)
    
    sdata.varproto += "#define NUM_LIMITS {0}\n".format(len(limits))
    if len(limits) > 0:
        sdata.varproto += "extern gpio_t limit[NUM_LIMITS];\n"
        sdata.vardefn += "gpio_t limit[NUM_LIMITS] = {{ \n{0} \n}};\n\n".format(",\n".join(blocks))

    homes = config["homes"]
    if len(homes) > len(sdata.axes):
        raise RuntimeError("More homing switches than defined axes.")
    blocks = [""] * len(homes)
    for i in range(len(homes)):
        h = homes[i]
        if not "axis" in h:
            raise RuntimeError("Couldn't find axis")
        if not "home" in h:
            raise RuntimeError("Couldn't find homing pin definition in homes item {0}".format(i))
        axis = h["axis"]
        pd = PinDefinition.parse("home[AXIS_{0}]".format(axis.upper()),h["home"])
        blocks[get_axis(h["axis"])] = "  // {0} axis\n  {1}".format(h["axis"],pd.defnexpr)
        sdata.itpindefs.append(pd)

    sdata.vardefn += "#define NUM_HOMES {0}\n".format(len(homes))
    if len(homes) > 0:
        sdata.varproto += "extern gpio_t home[NUM_HOMES];\n\n"
        sdata.vardefn += "gpio_t home[NUM_HOMES] = {{\n{0}\n}};\n\n".format(",\n".join(blocks))


def create_interrupt_handler (config,sdata):
    body = "\n\nvoid HAL_GPIO_EXTI_Callback (uint16_t pin) {\n\n"

    # First read in the value of each interrupt pin in this lineset
    for pd in sdata.itpindefs:
        body += Template(
"""  if (pin & GPIO_PIN_${bitnumber}) {
    GPIO_PinState curval = HAL_GPIO_ReadPin(GPIO${portletter},bit(${bitnumber}));
    if (${name}.shadow_input != curval) {
      // ${name} has changed here, so trigger an edge.
      // TODO ... some function call based on type=${type}

      // finally, shadow the actual value
      ${name}.shadow_input = curval;
    }
  }\n""").substitute(pd.__dict__)

    body += "\n}\n"
    sdata.vardefn += body


def lineset_from_handlername (name):
    '''This method computes a bitmask from an interrupt handler name according to STM's naming scheme for these handlers.'''
    pass


def create_code (config, outfn):
    global sdata
    parse_axes(config, sdata)
    create_system_defns(config, sdata)
    create_motor_defns(config, sdata)
    create_limit_defns(config, sdata)
    create_interrupt_handler(config, sdata)


def readfile(fn):
    with open (fn, "r") as f:
        data = f.readlines();
    return data


if __name__ == "__main__":
    if len(sys.argv) > 1:
        inputfn = sys.argv[1]
    
    config = parse_config(inputfn)
    sdata.template_c = readfile(template_cfn)
    sdata.template_h = readfile(template_hfn)

    create_code(config, "gpio.map.c")
    print(sdata.vardefines)
    print("\n-------\n")
    print(sdata.varproto)
    print("\n-------\n")
    print(sdata.vardefn)

