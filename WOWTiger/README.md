# Weta Workshop - WOW (World of Wearable Arts) Animatronic Tiger.

This is the source code I wrote that was used in the animatronic tiger build by Weta Workshop for the World or Wearable Arts show held in Wellington in 2016. A short film of the tiger is available here: https://www.facebook.com/WorldofWearableArt/videos/throwback-making-of-the-2016-wow-awards-show-tiger/2302859929994079/

I worked on the electronics design as well as developing and writing all the code (that you see here) used to make the tiger work. Unfortunately the film just says a "computer system" to make it work so this is a more in depth description. Weta Workshop creates magic with props and sets and so on but to them software is even more magic and it is something they don't really understand! 

It is a few years since I wrote all this so my description below might be off but the code is all there to look at if you want exact details.

The hardware was built around Dynamixel servos (http://www.robotis.us/dynamixel/) to move the facial features and tail. The main feature was the jaw but the eyes, eye lids, ears, cheeks and other parts of the face were all controllable. It included a Midi receiver which would read the Midi time signal used in the theatre to sync up the lights and sound track. It also included a RC receiver which was use to 'program' the Tiger and also give a manual override if needed. The manual override would only provide four channels of control but given that the jaw was the prominent feature that was enough. There was also an SD card which contained all the pre-recorded movement information.

Basically this is a simple record/playback system. We used the RC controller to manually operate the tiger while listening to the pre-recorded audio track. There were six tracks corresponding to the six times the tiger was on stage. We would manually puppeteer the different features and the code would record the movements. Because we couldn't do all the features at once we had to do multiple 'takes' where we would record one channel then play that back and add another over the top. We would start with the jaw as that was the dominant feature. It took a lot of recordings to build up the whole show and it took practice and many attempts to manually lip sync the jaw to the audio. I don't know how many times I have had to listen to Jemaine Clement sing 'What's up pussy cat"! 

Once we had built up a recording of all the different servo actions required you could then play back the result. The play back was synced to the Midi time signal so it would know exactly what point in the show we were at and it could start playing from that point. This got a little tricky as even with a Teensy as the controller is could be slow for the tiger to work out where it should be up to. I didn't have time to come up with a fast way to sync up the action so what I had to do effectively fast forward through the recorded data until it caught up. This was quite slow so in the code I would look at how far out of sync the tiger was then jump ahead that much. This had to be done iteratively since it was so slow by the time it caught up the tiger had moved on! You can see in the Play() function how SkipAhead() was used in a while loop in the code. Crude but it worked.

The tiger wasn't fully animatronic. The head and his paws were actually controlled by two operators who were both lovely women who are actually performers and acrobats. They had to perform inside the tiger on stage in the dark in very confined and hot conditions. I got to find out exactly what happened inside there as the tiger was experiencing some glitches during the production so I ended up inside it while it was live onstage with my laptop trying to debug what was happening. That was quite an experience! 

The issue turned out to be faulty wiring (not done by me!) causing the electronics to reset. The tiger then had to resync to the audio being played which could take several seconds as it needed to play 'catch up' as described above. While this happened the animatronics wouldn't be working although his head and paws being human operated would still move of course. The only features really visible to the audience was the jaw and tail anyway so most people probably wouldn't have noticed the glitch.

We did have a backup mode where my colleague could switch the tiger to full RC mode in which case he could then work the jaw manually using the RC transmitter. We actually used that feature in the show as they added an extra bit of dialogue for the tiger right at the end of the performance. It was too late to re-record that so my colleague would manually switch to the back up mode and remotely puppeteer the jaw for the final part.

Our back up back up plan was set off the theatre fire alarm :)

This IS NOT a running project. It requires very custom, one-off hardware (which no longer exists) and may use external libraries not included here. This code is provided purely for interested parties to examine code that was used in working animatronic devices used in the film industry.

## Authors

* **Simon Jansen** - [Asciimation](http://www.asciimation.co.nz)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments
