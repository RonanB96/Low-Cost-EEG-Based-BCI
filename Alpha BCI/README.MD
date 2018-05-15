# Alpha Wave Brain-Computer-Interface

This is folder contains a simpler BCI based off of Alpha waves which are in the range of 8-13Hz. The electrodes should be placed on the occipital lobe, connecting the measurement electrodes O1-O2 should yield a good signal.

Rather than the checkerboxes, it has two plain boxes. A "Yes" and a "No" box which change in size depending on the alpha wave activity of the user.
Alpha wave activity can be increased by the user closing their eyes and relaxing. This BCI was developed to demonstrate that this setup was capable of being used in a BCI. A lot of refinement could be used in this code as it was made from the SSVEP BCI code fairly quickly. It will be cleaned up in the future.

## Instructions

1. Build the circuit as shown in the circuit schematic
2. Import and build the Nucleo code to whichever ARM compatible IDE you wish. I used SW4STM32.
3. Flash it to the Nucleo

1. Run alphaBCI.py, you may need to change the serial port in this file.
2. See that the EEGScope window is receiving data. 
3. If it is, you can close the windows and connect up the electrodes.

The following instructions are for connecting the measurement electrodes between O1 and O2 and the Driven right leg (DRL) to the left/right mastoid

1. Connect the EEG cup electrodes to the positive and negative electrode leads (instructions for making the Low-Cost EEG cup electrodes I used can be found in the Electrodes Folder)
2. Connect an ECG electrode to the DRL lead
3. Remove the pad from the ECG pad electrode and place one behind either ear (mastoid)
4. (Optional) Apply Abrasive gel to Oz (I used Nuprep), this may improve signal quality
5. Fill the electrode cups up with conductive gel (I used Signa Electrode Gel)
6. Place the electrode cups on O1 and O2 and put on a swimming cap to hold them in place
7. Start alphaBCI.py and follow the instructions on screen

