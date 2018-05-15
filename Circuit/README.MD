# EEG Measurement Circuit

This folder contains the LTspice and a PDF of the circuits.
The EEG_sch files contains the EEG measuring circuit and the opt_iso_sch files contains the serial optoisolation circuit

## LTspice simulation

I've included some extra components and have the circuit setup to read them in automactically but I was not able to do this with the zener diode (BZX84C3V6L). To import this you'll need to copy the contents of BZX84C3V6L.lib to the end of LTspiceXVII/lib/cmp/standard.dio. Then it should work.

## Circuit Input
Below shows theoretical values for the circuit.

| Value           | Minimum Gain  | Maximum Gain  |
|:----------------|:-------------:| -------------:|
| Gain            | 51            | 25500         |
| Input Max pk-pk | 29.4mV        | 58.82uV       |
| Input resolution| 15.8uV        | 31.6nV        |
| Max DC Input    | +-500mV       | +-500mV       |

## Frequency Response

Simulated Driven right leg frequency to a common mode input of 200mV
![Driven Right Leg Frequency Response](drl_freq_response.png?raw=true)

Simulated Output of circuit to a differential input 50uV
![Output Frequency Response](output_freq_response.png?raw=true)

## Bill of Materials

A bill of materials can be found in [BOM.ods](BOM.ods).
Total cost of the circuit was €30.08. Cost per electrode was €0.70.
Extra channels can be added for €4.85 each as things should the DRL, Nucleo and optoisolation circuit can be reused

## The circuit can be used for other than EEG measurement

The circuit has been tested measuring an ECG signal which gave a very clean signal. It could also be used for EOG and EMG but the a cut-off frequency of the anti-alias will need to be increased.


