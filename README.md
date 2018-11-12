# Low Cost EEG Based Brain Computer Interface

The project uses a live EEG from the user to control a simplified keyboard using steady state visually evoked potentials.

On start-up, the user should stare at one of the checker boxes to record a baseline EEG. Once the checkerboxes start flashing, the user can start making selections. There are five flashing checkerboxes, each flashing at a different frequency. Each box also has some comma separated options above them, which can be selected
By the user looking and concentrating on it. The box will be highlighted and the options sub-divided between the checker boxes. This repeats until a single option is selected. If the option is a letter/number it will be displayed in the text box.

**The SSVEP BCI is not reliable right now**

In the circuit folder is a ltspice schematic for a single channel EEG measuring circuit which is measured by a Nucleo F303K8. The values are sent
to the brain-computer interface which processes the data.

A video overview of the circuit can be seen below  
<a href="http://www.youtube.com/watch?feature=player_embedded&v=Ilv_VNvS42w
" target="_blank"><img src="http://img.youtube.com/vi/Ilv_VNvS42w/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

A simple Alpha wave BCI can be found in the Alpha BCI folder and a video demonstrating can be seen below  
<a href="http://www.youtube.com/watch?feature=player_embedded&v=Ehdn_71upWc
" target="_blank"><img src="http://img.youtube.com/vi/Ehdn_71upWc/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

This was my final year project for my Electrical/Electronic Engineering Degree

## Getting Started

1. Build the circuit as shown in the circuit schematic
2. Import and build the Nucleo code to whichever ARM compatible IDE you wish. I used SW4STM32.
3. Flash it to the Nucleo

1. Run eegBCI.py, you may need to change the serial port in this file.
2. See that the EEGScope window is receiving data. 
3. If it is, you can close the windows and connect up the electrodes.

The following instructions are for connecting the measurement electrodes between 0z and the right/left mastoid and the Driven right leg (DRL) to the left/right mastoid

1. Connect an EEG cup electrode to the positive electrode lead (instructions for making the Low-Cost EEG cup electrodes I used can be found in the Electrodes Folder)
2. Connect an ECG electrode to the negative electrode and DRL lead
3. Remove the pads from the ECG pad electrodes and place one behind either ear (mastoid)
4. (Optional) Apply Abrasive gel to Oz (I used Nuprep), this may improve signal quality
5. Fill the electrode cup up with conductive gel (I used Signa Electrode Gel)
6. Place the electrode cup on Oz and put on a swimming cap to hold it in place
7. Start eegBCI.py and follow the instructions on screen

### Prerequisites

The BCI uses Python3 and all of the dependencies are listed in requirments.txt in the BCI folder. 
Run "pip install -r requirements.txt" within the folder

Nucleo code was created with STM32CubeMX and edited in SW4STM32 but can be imported into other ARM based IDE

## Contributing

TODO

## Authors

* **Ronan Byrne** - *Initial work* - [RonanB96](https://github.com/RonanB96) - [Blog](https://roboroblog.wordpress.com/)

## License

This project is licensed under the Apache 2 License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgements

* [Dr Ted Burke](https://batchloaf.wordpress.com/) for supervising the project and providing endless help and ideas

## Disclaimer
The designs for the EEG hardware and software shown in this repository have not been tested to comply with any medical device standards such as IEC 601. Therefore, any user which builds or uses the designs from this repository does so at their own risk. 

As of writing, the BCI designs in this repository are not very reliable so the software/hardware should not be modified to control dangerous machinery or provide a valid means of communication.
