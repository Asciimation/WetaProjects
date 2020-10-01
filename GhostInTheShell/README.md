# Weta Workshop - Ghost in the Shell props.

This is the source code I wrote that was used in props build by Weta Workshop used in the film Ghost in the Shell: https://www.imdb.com/title/tt1219827/

The three main projects were the shelling sequence skull, the animatronic Geisha mask and the Hanka bed prop. We spent months on making these work. Experimenting and designing the mechanisms and me writing all the code and building the controllers. Months and months of work ends up being seen on screen for probably less that 3 seconds total!

All of these props made use of Dynamixel servos (http://www.robotis.us/dynamixel/) and Teensy microcontrollers (https://www.pjrc.com/teensy/). I wrote a small library to wrap some of the Dynamixel functions and let me do things like acceleration/deceleration and so on.

### Shelling Sequence Skull

You can see this here: https://www.youtube.com/watch?v=F5amv-vqUFo

The skull was one prop and the skeleton another. Only the skull was animatronic and it consisted of a series of flaps that had to close in sequence to close up the skull as you see in the film. The eyes were also animatronic and moved down then into the prop using a very clever mechanism designed by my team lead. This was all done using Dynamixel servos and mostly cable drive as it actually had flap on flaps that all had to close.

I built and coded the controller used for it. Everything had to be adjustable to get the flaps to close exactly in the right sequence and at the right times. So the code provides a way to tweak all the settings. The controller used an LCD screen and a rotary encoder as well as buttons.

### Animatronic Geisha

You can see that here at about 8 minutes in: https://www.youtube.com/watch?v=KosBvDyWgnA

This is the fully animatronic Geisha head prop. There were also plain masks and another operated by remote control.

Again this is all done with Dynamixel servos in the head and my custom controller connected via cables to control it all (you can see the controller box behind the prop). This prop also consisted of flaps that needed to fly own quickly but also close slowly and perfectly. And we could open different sets of them as needed. These were all directly driven with linkages. The controller allowed everything to be finely adjusted to get it all to work properly. I believe the actual prop is still on display at the Weta Cave in Miramar, Wellington.
 
### Hanka bed prop

This was similar to the other two and was really a smaller version of the shelling sequence skull. I am not even sure it appeared in the film!

These ARE NOT running projects. They requires very custom, one-off hardware and may use external libraries not included here. This code is provided purely for interested parties to examine code that was used in working animatronic devices used in the film industry.

## Authors

* **Simon Jansen** - [Asciimation](http://www.asciimation.co.nz)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments
