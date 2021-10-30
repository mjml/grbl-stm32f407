#!/usr/bin/python3

import sys
import yaml
import re
import string
import getopt


from string import Template

inputfn = "pins.yaml"
template_cfn = "grbl/gpio_map.template.c"
template_hfn = "grbl/gpio_map.template.h"
output_cfn = "grbl/gpio_map.c"
output_hfn = "grbl/gpio_map.h"
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
    '''
    The pin's C symbol reference is given by self.name. This can be an array reference. You should be able to take address-of (&) of this name.
    When the pin is defined as an external interrupt input, self.type determines which preprocessor macro to invoke in response to an edge.
    The rest of the first group (mode,pull) are HAL string constants that determine physical mappings and parameters of the pin.

    The pu,pd,od,pp flags are indicators of the mode of the pin.

    The self.defnexpr is a curly-brace initializer of the gpio_t structure.
    '''
    name,type,port,pin,mode,pull = ("", "", "", "", "", "")
    portletter, bitnumber = ("", "")
    it, input = (False, False)
    pu, pd, od, pp = ("", "", "", "")
    defnexpr = ""

    def parse (name,pinstr,type="out"):
        self = PinDefinition()
        self.name = name
        self.type = type
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
        it = (name in interrupt_names) or ("!" in name)
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
        elif self.name == "alm":
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
    sdata.vardefines += "#define NUM_AXES {0}\n".format(len(sdata.axes))
    for i,axis in enumerate(sdata.axes):
        sdata.vardefines += "#define AXIS_{0} {1}\n".format(axis.upper(),i)


def create_system_defns (config, sdata):
    '''
    Creates the required reset input pin, and optional cycle_start, feed_hold, and safety_door input pins.
    '''
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
        resetpin = PinDefinition.parse("reset",sysblock["reset"],type="system")
        sdata.itpindefs.append(resetpin)
        sdata.vardefn += "gpio_t reset = {0};\n".format(resetpin.defnexpr)
    else:
        raise RuntimeError("No reset pin found in system block.")
    
    if "cycle_start" in sysblock:
        sdata.vardefines += "#define ENABLE_CYCLE_START\n"
        sdata.varproto += "extern gpio_t cycle_start;\n"
        cspin = PinDefinition.parse("cycle_start",sysblock["cycle_start"],type="system")
        sdata.itpindefs.append(cspin)
        sdata.vardefn += "gpio_t cycle_start = {0};\n".format(cspin.defnexpr)

    if "feed_hold" in sysblock:
        sdata.vardefines += "#define ENABLE_FEED_HOLD\n"
        sdata.varproto += "extern gpio_t feed_hold;\n"
        fhpin = PinDefinition.parse("feed_hold",sysblock["feed_hold"],type="system")
        sdata.itpindefs.append(fhpin)
        sdata.vardefn += "gpio_t feed_hold = {0};\n".format(fhpin.defnexpr)

    if "safety_door" in sysblock:
        sdata.vardefines += "#define ENABLE_SAFETY_DOOR\n"
        sdata.vardefines += "#define ENABLE_SAFETY_DOOR_INPUT_PIN\n"
        sdata.varproto += "extern gpio_t safety_door;\n"
        sdpin = PinDefinition.parse("safety_door",sysblock["safety_door"],type="system")
        sdata.itpindefs.append(sdpin)
        sdata.vardefn += "gpio_t safety_door = {0};\n".format(sdpin.defnexpr)
    elif "feed_hold" in sysblock:
        sdata.vardefines += "#define ENABLE_SAFETY_DOOR\n"
        sdata.varproto += "extern gpio_t safety_door;\n"
        sdata.vardefn += "gpio_t safety_door = {0};\n".format(fhpin.defnexpr)

    sdata.vardefn += "\n"
    sdata.varproto += "\n"


def create_coolant_defns(config, sdata):
    coolant = config["coolant"]
    title = "///// Coolant /////////////////////////\n"
    if "flood" not in coolant:
        raise "Expected coolant/flood pin mapping not found."
        return

    sdata.varproto   += title
    sdata.vardefn    += title
    for k,s in coolant.items():
        pd = PinDefinition.parse(k,s)
        if k == "mist":
            sdata.vardefines += "#define ENABLE_M7"
        sdata.varproto += "extern gpio_t {0};\n".format(k)
        sdata.vardefn  += "gpio_t {0} = {1};\n".format(k,pd.defnexpr)
    
    sdata.vardefn  += "\n"
    sdata.varproto += "\n"

def create_probe_defns (config, sdata):
    '''
    The standard grbl probe is named 'contact' or 'grbl', while we use a different pin for ATC tool length probe.
    This is so that we have the freedom to wire the latter as NO or NC while the grbl probe is always NO.
    '''
    if "probes" not in config:
        return

    probes = config["probes"]
    title = "///// Probe{0} /////////////////////////\n".format("s" if len(probes) > 1 else "")
    sdata.varproto += title
    sdata.vardefn += title

    for k,s in probes.items():
        probe = PinDefinition.parse(k, s, type="probe")
        # This related to the ATC tool length sensor and its own code
        if k.startswith("atc"):
            sdata.vardefines += "#define ENABLE_ATC_TOOL_LENGTH_PROBE\n"
        # This is the standard grbl tool probe
        if k == "contact" or k == "grbl":
            sdata.vardefines += "#define ENABLE_CONTACT_PROBE\n"
        sdata.varproto += "extern gpio_t {0};\n".format(probe.name)
        sdata.itpindefs.append(probe)
        sdata.vardefn += "gpio_t {0} = {1};\n".format(probe.name, probe.defnexpr)

    sdata.varproto += "\n"
    sdata.vardefn += "\n"


def create_motor_defns (config, sdata):
    '''
    Configures the stepper motor pins with optional encoder alarm input.
    '''
    title = "///// Motors /////////////////////////\n"
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
                pd = PinDefinition.parse(s, m[s], type="out")
                gpiodefs.append("    " + pd.defnexpr)
            if s == "alm":
                encoder_input_found = True
                sdata.itpindefs.append(pd)
                pd.name = "motor[AXIS_{0}].alm".format(axis.upper())

        blocks[get_axis(axis)] = "  {{\n    // {0} axis\n{1}\n  }}".format(axis,",\n".join(gpiodefs))
    
    if encoder_input_found:
        sdata.vardefines += "#define STEPPER_ENCODER_ALARM\n"

    sdata.varproto += "#define NUM_MOTORS {0}\n".format(len(motors))
    sdata.varproto += "extern stepper_motor_t motor[NUM_MOTORS];\n\n"
    sdata.vardefn += "stepper_motor_t motor[NUM_MOTORS] = {{\n{0} \n}};".format((',\n').join(blocks))
    sdata.vardefn += "\n\n"


def create_limit_defns (config, sdata):
    '''
    Configures limit and homing pins.
    '''
    title = "///// Limits & Homing /////////////////\n"
    sdata.varproto += title
    sdata.vardefn += title
    
    # Note that home interrupts need to be called first, since a single pin servicing both homes and limits
    #  will fire both interrupts, and limit code may put state into E-Stop, whereas the home interrupt 
    #  just needs to check if we're in a homing routine.
    if "homes" in config:
        homes = config["homes"]
        sdata.vardefines += "#define SEPARATE_HOME_DEFNS\n"
        if len(homes) != len(sdata.axes):
            raise RuntimeError("Number of homing switches does not equal the number of axes.")
        blocks = [""] * len(homes)
        homesw_mask = 0
        for i in range(len(homes)):
            h = homes[i]
            if not "axis" in h:
                raise RuntimeError("Couldn't find axis")
            if not "home" in h:
                raise RuntimeError("Couldn't find homing pin definition in homes item {0}".format(i))
            axis = h["axis"]
            ai = get_axis(axis)
            homesw_mask = homesw_mask | (0x1 << ai)
            pd = PinDefinition.parse("home[AXIS_{0}]".format(axis.upper()),h["home"],type="home")
            blocks[ai] = "  // {0} axis\n  {1}".format(h["axis"],pd.defnexpr)
            sdata.itpindefs.append(pd)
        
        sdata.varproto += "#define NUM_HOMES NUM_AXES\n" 
        sdata.varproto += "#define HOMESW_MASK {0}\n".format(homesw_mask)
        sdata.varproto += "extern gpio_t home[NUM_HOMES];\n"
        sdata.vardefn  += "gpio_t home[NUM_HOMES] = {{ \n{0} \n}};\n\n".format(",\n".join(blocks))

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
        pd = PinDefinition.parse("limit[AXIS_{0}]".format(axis.upper()),l["limit"],type="limit")
        blocks[get_axis(l["axis"])] = "  // {0} axis\n  {1}".format(l["axis"],pd.defnexpr)
        sdata.itpindefs.append(pd)
    
    sdata.varproto += "#define NUM_LIMITS {0}\n".format(len(limits))
    if len(limits) > 0:
        sdata.varproto += "extern gpio_t limit[NUM_LIMITS];\n\n"
        sdata.vardefn += "gpio_t limit[NUM_LIMITS] = {{ \n{0} \n}};\n\n".format(",\n".join(blocks))

  
def handler_from_pindef (config,sdata,pd):
    '''
    This method simply generates a bunch of preprocessor macro calls that can be defined within the source code itself.
    That handler receives a pointer to a gpio_t structure identifying the pin that triggered the interrupt.
    '''
    if pd.type == "limit":
        return "__LIMIT_HANDLER__(&{0})".format(pd.name)
    elif pd.type == "home":
        return "__HOME_HANDLER__(&{0})".format(pd.name)
    elif pd.type == "encoder_alarm":
        return "__ENCODER_ALARM_HANDLER__(&{0})".format(pd.name)
    elif pd.type == "system":
        return "__SYSTEM_HANDLER__(&{0})".format(pd.name)
    elif pd.type == "probe":
        return "__PROBE_HANDLER__(&{0})".format(pd.name)
    else:
        raise Exception("Unknown interrupt pin type. Cannot generate handler")


def create_interrupt_handler (config,sdata):
    body = "///// Chained Interrupt Handler called by STM32 HAL ///////////////\n\n"
    body += "/** Dispatches to individual pin handlers and modules via preprocessor #defines */\n"
    body += "void HAL_GPIO_EXTI_Callback (uint16_t pin) {\n\n"

    # First read in the value of each interrupt pin in this lineset
    for pd in sdata.itpindefs:
        params = { **pd.__dict__, 'handler':handler_from_pindef(config,sdata,pd) }
        body += Template(
"""  if (pin & GPIO_PIN_${bitnumber}) {
    GPIO_PinState curval = HAL_GPIO_ReadPin(GPIO${portletter},bit(${bitnumber}));
    if (${name}.shadow_input != curval) {
      // shadow the actual value
      ${name}.shadow_input = curval;
      // Call the handler for ${name}
      ${handler};
    }
  }\n""").substitute(params)

    body += "\n}\n"
    sdata.vardefn += body


def create_spindle_defns(config, sdata):
    spindle = config["spindle"]
    title = "///// Spindle /////////////////////////\n"
    sdata.varproto   += title
    sdata.vardefn    += title
    if "ena" not in spindle:
        raise Exception("Couldn't find \"ena\" pin definition key/value under spindle section.")
    if "dir" in spindle:
        sdata.varproto += "#define SPINDLE_DIR\n"
    if "pwm" in spindle:
        sdata.varproto += "#define VARIABLE_SPINDLE\n"
        (timerbase, channel) = spindle["tim"].split(' ')
        channel = "TIM_CHANNEL_" + ''.join(filter(str.isdigit, channel))
        sdata.varproto += "#define SPINDLE_TIM      {0}\n#define SPINDLE_TIM_CH   {1}\n".format(timerbase, channel)
    for k,s in spindle.items():
        if k=="tim" or k=="pwm":
            continue
        pd = PinDefinition.parse(k,s)
        sdata.varproto += "extern gpio_t " + "spindle_" + pd.name + ";\n"
        sdata.vardefn  += "gpio_t " + "spindle_" + pd.name + " = " + pd.defnexpr +";\n";
    sdata.varproto += "\n"
    sdata.vardefn += "\n"


def create_code (config):
    global sdata
    parse_axes(config, sdata)
    create_system_defns(config, sdata)
    create_probe_defns(config,sdata)
    create_motor_defns(config, sdata)
    create_limit_defns(config, sdata)
    create_spindle_defns(config, sdata)
    create_coolant_defns(config, sdata)
    create_interrupt_handler(config, sdata)


def readfile(fn):
    with open (fn, "r") as f:
        data = f.read();
    return data


def writefile(fn,data):
    with open (fn, "w") as f:
        f.write(data)
    return


if __name__ == "__main__":
    config = parse_config(inputfn)
    sdata.template_c = readfile(template_cfn)
    sdata.template_h = readfile(template_hfn)

    create_code(config)
    
    opts,args = getopt.getopt(sys.argv[1:],"n")
    opts = dict(opts)
    if ('-n' in opts):
        print("Test mode.\n")
        print(sdata.vardefines)
        print("\n-------\n")
        print(sdata.varproto)
        print("\n-------\n")
        print(sdata.vardefn)
    else:
        ct = Template(sdata.template_c).substitute(sdata.__dict__)
        ch = Template(sdata.template_h).substitute(sdata.__dict__)
        writefile(output_cfn, ct)
        writefile(output_hfn, ch)
        

