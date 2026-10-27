# Physics backend for ArduPilot JSON SITL using the Dyad quadcopter model.
#
#   julia --project=. scripts/run_ardupilot_sitl.jl
#
# Then, on the ArduPilot side (same machine, or replace 127.0.0.1 by this
# machine's IP):
#   sim_vehicle.py -v ArduCopter -f quad --model JSON:127.0.0.1 \
#       --add-param-file=<path>/scripts/quad450.parm --console --map
#
# Environment overrides (optional):
#   SITL_PORT   UDP port to listen on            (default 9002)
#   SITL_WIND   constant wind "N,E,D" in m/s     (default "0,0,0")
#   SITL_YAW0   initial heading in degrees       (default 0)

import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

using quadcopter
using Sockets: @ip_str

port = parse(Int, get(ENV, "SITL_PORT", "9002"))
wind_ned = parse.(Float64, split(get(ENV, "SITL_WIND", "0,0,0"), ","))
yaw0 = deg2rad(parse(Float64, get(ENV, "SITL_YAW0", "0")))

quadcopter.run_ardupilot_bridge(; port, bind_ip = ip"0.0.0.0", wind_ned, yaw0)
