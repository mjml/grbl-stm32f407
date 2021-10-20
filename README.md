![Logo](/doc/images/logo.svg|scale=50)

Micromachine is a CNC firmware forked from gnea/grbl.

Instead of an Arduino, it runs on the STM32F407 chip, running at 168MHz.

This allows it to provide faster pulse rates, and to work with larger industrial grade stepper drivers (e.g. Leadshine and similar).

The plan is to implement support for an ATC spindle and to accept commands from a CAM program derived from grbl-friendly Candle.

![System Architecture](/doc/SystemWiring.svg)
