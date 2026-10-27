module quadcopter

include("../generated/module.jl")

# ArduPilot JSON SITL physics backend
include("sitl_bridge.jl")

end # module quadcopter