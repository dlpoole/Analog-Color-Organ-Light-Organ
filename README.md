# -Analog-Color-Organ-Light-Organ
3-Channel “Analog” Color Organ/Light Organ for Arduino DUE and WS2811 Strings

This sketch emulates an analog system patented to Frederic L. Way as US 3,018,683 and US
3,181,015 in 1962 and 1965, claiming bandpass filters and mechanically controlled resistive
dimmers or Silicon Controlled Rectifiers. Way’s patents were assigned to Mobilcolor, Inc,
which leased equipment based on them to radio stations including Omaha and Indianapolis,
where it was used for holiday “Carol Tree” displays.

Improvements: This implementation adds a fast AGC, digital filtering, full wave peak
detection, dB scaling, smoothing time constants, and mapping of the three band dB converted
results to WS2811 RGB intensities over a given dynamic range. Cascaded first order filters
are used to make 10KHz sampling feasible within the CPU capability of the Arduino DUE.
The filter crossovers are at 60Hz and 1200Hz instead of Way’s 540Hz and 1800Hz, but these
and all other “analog” parameters are named constants in this sketch so as to be easily tuned
to taste.

Effects & Animations: As alternatives to music input, an IR remote can be used to select a
uniform color of arbitrary hue, saturation and brightness, random colors, or a rainbow along
successive LEDs with selectable saturation and brightness. The hue can be varied or the
rainbow trickled and a sparkle effect can be added and sped up or slowed. Those effects
could be forked and recompiled for Arduino UNO with appropriate changes in pin
assignments. Only the color organ function requires the greater CPU horsepower of the DUE.

Libraries: This sketch requires the FastLED library from http://fastled.io and a fork of the
library irlib2 from https: github.com/cyborg5/IRLib2 with unreferenced headers and code for
IRLLibRecv removed since it throws errors if compiled for SAM3X8E. The forked library,
renamed IRLibPCISamX, can be carried along with the files for this sketch and placed in the
/libraries folder associated with the IDE, which then must be restarted to recognize it.

DUE Noise Issue: Later shipments of DUE model A000062 used a MPM3620 switching
regulator in lieu of the LM2734Y of the reference design. The MPM3620 operates at at 2MHz
so as to avoid a discrete 100uH inductor; this, however, results in substantial loop currents
within the regulator circuit which introduce random noise in the low LS 4 to 6 bits of the ADC
causing flicker in the LEDs when quiet. This cannot be decoupled external to the
microcontroller. With these boards, AGC_BOOST must be reduced to avoid noise flicker and
at the expense of dynamic range and requiring a 1V RMS audio signal.

DUE Clones: The A00062 Due reportedly stood discontinued when this sketch
was coded, but several DUE R3 clones were on the market using the LM2734Y and discrete
inductor of the reference design. These were noise free, but lacked resistor R99 of the
reference design, which was introduced after the schematic was published, to ground the
gate on MOSFET T3 during the startup of the SAM3X8E and the USB interface which
controls it. Without a rework to add this 10K resistor between the source and drain of T3, an
R3 clone might start in ERASE mode (though no erase was observed) unless powered OFF
and back ON before charge leaks off the floating MOSFET gate.

Hardware Requirements: The required interfaces are shown the accompaning interface schematic and
consist of a stereo channel summer, a capacitive decouple, and 1.65V bias network; a 40109
level shifter to adapt the 3.3V logic HIGH of Arduino to the 5V data requirement of WS2811
strings; a three pin interface for a TSOP34438 38KHz 3.3V IR Receiver, a 12 V supply and
input connector of adequate current capability for the number of strings used, and 12V and
data outputs for as many WS2811 strings as are required. The sketch assumes 4 such
strings with the 5V serial data delivered on pins 25-28. For the color organ/light organ effect,
all strings are equivalent; for the rainbow effect, the strings run in consecutive order.
WS2811 strings are available from a few domestic importers and Chinese sources. Although
the strings typically have a male connector and pigtail at one end and a female connector
molded to the other so as to suggest daisy-chaining them, at least some ship with the
equivalent of 21 AWG stranded wire, the resistance of which is too great to support more than
50 LED clusters, which at full white, total 2.5 Amperes. This isn’t noticeable across a single
string because there is some current compliance within the WS2811, but it will become
noticeable with two strings. Some resellers claim strings with 18 AWG wire, in which case the
number of interfaces might be relaxed, but the #define NUMBER_STRINGS 4 and #define
LEDS_PER_STRING 50 would require change.

Infrared Remote: Although a few other decoders are anticipated in the #includes and could
be handled in a fork of the sketch, this sketch assumes NEC codes such as that sent by the
Adafruit Mini Remote. This relies on the NEC autoresume feature which uses the repeat
(0xffffffff) code as an end-of-frame. The NEC decoder rapidly decodes the repeat frames,
necessitating a count of 5 repeats before accepting a held key as a repeat. An overlay,
RemoteCover.odg can be printed with LibreOffice Write and pasted on the remote to show
the remote key assignments.

Audio Sources: Nearly any audio source can be used, typically the “record” or “line” output
of a stereo or the headphone jack of a music player, cell phone or tablet. A Y-cable may be
needed to split the signal between the color organ and a boom box or stereo because
plugging into a phone or tablet’s earphone jack disables its internal speaker. A Bluetooth
receiver “dongle” paired to the cell phone or tablet can also be used, although the Y-cable is
still needed to split the audio signal for a separate amplifier and speaker since Bluetooth
devices operate in pairs and there is some latency.

Remote Control: The IR sensor can be mounted on a small protoboard strip at the end of a
4’ cord with a 3mm (1/8”) female stereo plug. Its lens is a bubble on one side of the dongle,
which can be hidden in plain sight as long it roughly faces the remote.
When power is applied, the LED strings will light solid violet, blue, green, and yellow, in
numerical order to confirm which string is which. After two seconds, the color organ function
starts automatically. If there is an audio signal, the LEDs will begin to follow it instantly;
otherwise, the LEDs will go out. If there are no other lights in the room, this may be a bit stark,
so you can opt for a “Quiet Color” which will take effect whenever the music stops.

The Quiet Color is adjustable by pressing the Quiet Color button. This stops color organ
operation even with audio and shows the Quiet Color, initially blacked out. Press the Bright
button to start with a dim white, which you can further Brighten, Deepen into a blue, then
change that color by pressing a Hue button. Return to color organ by pressing the Color
Organ button. The new Quiet Color will remain in effect for as long as the controller is
plugged into power.

Volume Setting: The color organ self-adjusts to the level of the input signal over a fairly wide
range, but if your volume setting or input signal is too low, the colors will not reach full
brightness. If your volume setting is too high, no damage occurs but only soft passages will
show color and loud passages will turn on all colors at once, producing mostly pastels.
Whether you have control of this depends on the type of input source you are using. Line and
record outputs on a stereo are best because they deliver a constant level independent of your
volume setting. The effect can be visualized by comparing the three filter plots with
DYNAMIC_RANGE, the LED off threshold and a modifiable named parameter in the sketch.
The color organ can be turned OFF and back ON with the ON/OFF button. When turned
back ON, the color organ will resume as it was when turned OFF, including the Quiet Color as
long as the unit hasn’t been unplugged while OFF.

ALTERNATIVE EFFECTS

As an alternative to music input, the Remote can be used to set virtually any uniform color of
arbitrary hue, saturation and brightness or one of the animations described below.
The Color-Selection Method is Hue, Saturation, Brightness, or HSB. Hue is one of the
“pure” colors around the edge of the color space diagram on the top of the remote.. it
corresponds to the angular position around that circle, with red at the 12:00 position; blue at
8:00, and green at 4:00. White appears at the center of the space and Saturation is the radial
distance from white, so halfway out towards blue is “light blue,” etc. Brightness is the overall
brightness of the display.

A Uniform Color can be selected by pressing the Solid Colors button. Pressing it a second
time will rotate around the hue circle shown on the remote; pressing it again and again will
slow the rotation until a blink signals that its as slow as it can go.

A Rainbow can be selected by pressing the Rainbow button. Pressing Rainbow a second
time will rotate it around the hue circle shown on the remote, with the effect that it appears to
slide along the strings (or trickle down a tree.) Pressing Rainbow again and again will slow
the trickle until a blink signals that its as slow as it can go.

Random Colors can be selected by pressing the Random Colors button. Pressing it a
second time will start changing colors randomly. Pressing Random Colors again and again
will slow the changes until a blink signals that its as slow as it can go.

Sparkles can be added to any other effect by pressing the Sparkle button. Pressing it a
second time will start changing colors randomly. Pressing Sparkle again and again will slow
sparkling to the point you will barely see them. Sparkles can be turned OFF by pressing the
No Sparkle Button.

Trickle Down Sparkle can be triggered with the Trickle Button. Pressing Trickle again and
again will slow it down until a blink signals that its as slow as it can go.
The controller can be turned OFF and back ON with the ON/OFF button. When turned ON,
the controller will resume the color organ or whatever special effect was running when it was
turned OFF.
