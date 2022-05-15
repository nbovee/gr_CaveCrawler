# CaveCrawler

A video of the run from the accompanying log can be found here: 

We can see that the numerical implementation of SLAM with A* is extremely successful, with accurate mapping until environmental 
factors of slip and memory limitations on the Spike brick caused an error cascade. The errors appearance at the end of our trial 
is due to the restrictive angular offset search implemented. If SLAM methods assume less deflection than they actually 
encounter, then they are guaranteed to lose position over time. If we checked each potential offset up to say, 60 degrees, these 
issues are within the capability of the SLAM program to correct. However, each additional angle we inspect adds to our processing 
time, and standing still for 45 minutes filming was not going to happen at 3am. Secondary effects enhancing this error are the 
inconsistent friction of the carpet, and the inability of ultrasonic sensors to accurately detect surfaces they are not normal to.

Licensed Creative Commons Attribution-NonCommercial 4.0
https://creativecommons.org/licenses/by-nc/4.0

One of the best A* explanations I have found is at https://www.redblobgames.com/pathfinding/a-star/introduction.html
Minor portions of the code are structured after their implementation examples.
