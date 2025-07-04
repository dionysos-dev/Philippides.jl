using MeshCat

# target = "LIPM_closed"
# target = "LIPM_open"
# target = "Closed-dist_tier_g"
# target = "CLOSED_DIST_TIERSDEG_PULLED"
target = "enhanced_urdf_first_approach"
localdir = @__DIR__
targetpath = joinpath(localdir, target)

MeshCat.convert_frames_to_video(targetpath  * ".tar", targetpath  * ".mp4")