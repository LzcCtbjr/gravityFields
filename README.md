# gravityFields: A generative art project about Gravity

![example image](./out/example.png)

gravityFields simulates "gravity" between points, and shows you three different visualizations of the paths these points take.

The reason "gravity" is in quotes is because it doesn't work the same way as standard gravity. When looking at any two points:
- if they are "too close", they will repel each other
- if they are "too far apart", they won't act on each other
- and if they are between these two thresholds, they will attract one another

When this simulation is run for long enough, the points will always arrange in a gridlike pattern. 

Dependencies: cv2, random, numpy

**TODO:**
- allow passing the threshold parameters through the command line
- (?) optimize the saving to video
- (?) add a new class of point that has a fixed position, but still acts on the other points