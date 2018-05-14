# EEG Measurement Circuit

This folder contains the LTspice and a PDF of the circuit. 

## Circuit Input
Below shows theoretical values for the circuit
| Value           | Minimum Gain  | Maximum Gain  |
| ----------------|:-------------:| -------------:|
| Gain            | 51            | 25500         |
| Input Max pk-pk | 29.4mV        | 58.82uV       |
| Input resolution| 15.8uV        | 31.6nV        |
| Max DC Input    | +-500mV       | +-500mV       |

## Frequency Response

Simulated Driven right leg frequency to a common mode input of 200mV
![Driven Right Leg Frequency Response](/drl_freq_response.png?raw=true)

Simulated Output of circuit to a differential input 50uV
![Output Frequency Response](/output_freq_response.png?raw=true)

## The circuit can be used for other than EEG measurement

The circuit has been tested measuring an ECG signal which gave a very clean signal. It could also be used for EOG and EMG but the a cut-off frequency of the anti-alias will need to be increased.


