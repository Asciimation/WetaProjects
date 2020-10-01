# Weta Workshop - Bug lab.

This is the source code I wrote that was used in the Bugs museum exhibition build by Weta Workshop for the Te Papa in Wellington, New Zealand.

Some footage of the exhibition is available here: https://www.youtube.com/playlist?list=PLbjj-GAgB5ySzIo8A_8lY8PSV7URfWhLX

I worked on the electronics design as well as developing and writing all the code (that you see here) used for various parts of the exhibition.

There were four parts I worked on:

	- BeeRubbing game
	- Cockroach game
	- Beetle slide
	- Zoetrope
	
### Bee rubbing game	

This was a simple game which required a group of people to vigorously rub six pads to 'generate heat' to raise a temperature scale to the point where the the game was won. It was to simulate how certain honey bees work together in the hive to generate heat just enough to kill invading wasps but not so high as to kill themselves. The code just read six sensors to detect the rubbing and added all the values to output a DMX value fed into the rest of the electronics. It included a 'cool down' period. Very simple but it worked well.

### Cockroach game

Another simple game where there were two giant cockroach heads and the player had to insert a stick (a stinger) into the brain and hit the right spot to zombify the cockroach.  There were four spots, 3 bad and 1 good. Different sounds and light effects were played on hitting the spots. Again very simple.

### Beetle slide

This was a giant fibreglass slide you could slide down which would light up and play a farting noise as you exited the bottom. Popular with kids! Very simple coe to run some lights, some sounds and read some sensors then output DMX signals.

### Zoetrope

This was the most complicated part of the bugs exhibition in terms of mechanics, electronics and code. A zoetrope is a device used to animate a static scene by rotating and showing the viewer a sequence of scenes one at a time, each with a small movement in them from the last. Our Zoetrope used 3D printed models on a rotating platform illuminated by strobe lights. It used an industrial Arduino Mega to drive digital servos to control the spinning and lifting of the Zoetrope dome as well as trigger the strobe lights. It required the handling of things like acceleration/deceleration and accurate timings and so on as well as safety switches, limit switches and so on.
	

These ARE NOT running projects. They requires very custom, one-off hardware (which no longer exists probably) and may use external libraries not included here. This code is provided purely for interested parties to examine code that was used in working animatronic devices used in an interactive museum exhibition.

## Authors

* **Simon Jansen** - [Asciimation](http://www.asciimation.co.nz)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments
