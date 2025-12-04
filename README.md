# deformable-pov-display
TODO
weight of 139 LEDS + pitch = 22gm
pixel pitch weight = 22gm/139pixels = 0.15827338129gm
centrifugal force @380RPM and 300mm on one pixel pitch = 0.07519N (7.5gmF)

\
235RPM -> 205px (380px no pitch)
400RPM -> 120px (225px no pitch)
min_hole = 205.78268mm (D) or 102.89134mm (R)
min_height = 200mm 
min_length = 220mm 
extension_height_stage_1 = 170mm
extension_height_stage_2 = 155mm
# video from image
ffmpeg -loop 1 -i font-transparent.png -c:v libx264 -t 15 -pix_fmt yuv420p -vf scale=144:144 font-transparent.mp4
ffmpeg -loop 1 -i dual-chessboard-bis.jpg -c:v libx264 -t 15 -profile:v high444 dual-chessboard-bis.mp4

# Applications

## Planet Size Explorer
-   mars: H/7px, L0/7+14px, L1/7+0px

-   venus: H/42px, L0/42+26px, L1/42+0px

-   earth: H/44px, L0/44+30px, L1/44+0px    