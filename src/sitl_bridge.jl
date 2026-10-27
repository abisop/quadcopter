# ArduPilot JSON SITL bridge for the `QuadcopterSITL` Dyad model.
#
# Protocol (ArduPilot libraries/SITL/SIM_JSON):
#   SITL -> physics, UDP binary, little endian:
#       uint16 magic (18458: 16 channels, 29569: 32 channels)
#       uint16 frame_rate, uint32 frame_count, uint16 pwm[16 or 32]
#   physics -> SITL, UDP text: "\n{json}\n" with timestamp, imu.gyro,
#       imu.accel_body, position (NED), velocity (NED), quaternion / attitude
#       and the optional battery {voltage, current}.
#
# The physics side listens on port 9002 and replies to the sender address.
# Every frame: apply PWM 1..4 (zero-order hold), advance the model by
# 1/frame_rate, reply with the new state (lock-step).

# Import only what is needed: `Sockets.connect` would clash with MTK's `connect`
import Sockets
using Sockets: UDPSocket, recvfrom, send, @ip_str
import JSON
import OrdinaryDiffEqDefault
using ModelingToolkit: ODEProblem, setp, getu
using DyadInterface: simplify_model

const SITL_MAGIC_16 = 18458
const SITL_MAGIC_32 = 29569

"One decoded SITL output frame."
struct SITLFrame
    frame_rate::Int
    frame_count::Int
    pwm::Vector{Float64}
end

"""
    parse_sitl_frame(data) -> Union{SITLFrame, Nothing}

Decode an ArduPilot SITL servo-output packet (16- or 32-channel variant).
Returns `nothing` for packets with an unknown magic or a wrong length.
"""
function parse_sitl_frame(data::AbstractVector{UInt8})
    n = length(data)
    n < 8 && return nothing
    rd16(i) = Int(UInt16(data[i]) | (UInt16(data[i+1]) << 8))
    rd32(i) = Int(UInt32(data[i]) | (UInt32(data[i+1]) << 8) | (UInt32(data[i+2]) << 16) | (UInt32(data[i+3]) << 24))
    magic = rd16(1)
    nch = magic == SITL_MAGIC_16 ? 16 : magic == SITL_MAGIC_32 ? 32 : 0
    (nch == 0 || n < 8 + 2nch) && return nothing
    pwm = Float64[rd16(9 + 2(k - 1)) for k in 1:nch]
    return SITLFrame(rd16(3), rd32(5), pwm)
end

"""
    encode_sitl_frame(frame_count, pwm; frame_rate = 400) -> Vector{UInt8}

Encode a SITL servo-output packet (what ArduPilot sends). Used for testing the
bridge without ArduPilot.
"""
function encode_sitl_frame(frame_count::Integer, pwm::AbstractVector; frame_rate::Integer = 400)
    nch = length(pwm) <= 16 ? 16 : 32
    magic = nch == 16 ? SITL_MAGIC_16 : SITL_MAGIC_32
    buf = IOBuffer()
    write(buf, htol(UInt16(magic)), htol(UInt16(frame_rate)), htol(UInt32(frame_count)))
    for k in 1:nch
        write(buf, htol(UInt16(k <= length(pwm) ? round(Int, pwm[k]) : 0)))
    end
    return take!(buf)
end

"Stepping wrapper around the compiled `QuadcopterSITL` model."
mutable struct QuadSim
    sys::Any
    prob::Any
    integ::Any
    alg::Any
    integ_kwargs::NamedTuple
    setpwm::Any
    getstate::Any
end

"""
    build_quad_sim(; alg = Rodas5P(), wind_ned = [0, 0, 0], yaw0 = 0.0,
                     abstol = 1e-6, reltol = 1e-4) -> QuadSim

Compile `QuadcopterSITL`, build the ODE problem with the vehicle resting on its
landing gear (motors stopped, battery full) and create a stepping integrator.
`wind_ned` is a constant wind (m/s, NED); `yaw0` the initial heading (rad).
"""
function build_quad_sim(; alg = OrdinaryDiffEqDefault.Rodas5P(), wind_ned = [0.0, 0.0, 0.0],
                        yaw0 = 0.0, abstol = 1e-6, reltol = 1e-4)
    # BodyPort carries more potentials than flows by design (like Frame3D);
    # silence MTK's generic connector-balance warning while building.
    sys = Base.CoreLogging.with_logger(Base.CoreLogging.SimpleLogger(stderr, Base.CoreLogging.Error)) do
        simplify_model(QuadcopterSITL(; name = :sitl))
    end
    q = sys.quad
    op = Dict(sys.pwm_cmd => fill(1000.0, 4), q.wind_ned => collect(Float64, wind_ned), q.yaw0 => Float64(yaw0))
    prob = ODEProblem(sys, op, (0.0, 1e9))
    integ_kwargs = (; save_everystep = false, save_start = false, save_end = false,
                    dense = false, abstol, reltol)
    integ = OrdinaryDiffEqDefault.init(prob, alg; integ_kwargs...)
    setpwm = setp(sys, sys.pwm_cmd)
    getstate = getu(sys, [q.airframe.w, q.airframe.accel_body, q.airframe.r_ned, q.airframe.v_ned,
                          q.airframe.q, q.airframe.rpy, q.battery.v, q.battery.I])
    return QuadSim(sys, prob, integ, alg, integ_kwargs, setpwm, getstate)
end

"Restart the simulation from the initial state (vehicle on the ground)."
function reset_sim!(sim::QuadSim)
    sim.integ = OrdinaryDiffEqDefault.init(sim.prob, sim.alg; sim.integ_kwargs...)
    return sim
end

"""
    step_sim!(sim, pwm4, dt)

Hold the four motor PWM values (microseconds) for `dt` seconds.
"""
function step_sim!(sim::QuadSim, pwm4::AbstractVector, dt::Real)
    sim.setpwm(sim.integ, Float64[pwm4[1], pwm4[2], pwm4[3], pwm4[4]])
    OrdinaryDiffEqDefault.u_modified!(sim.integ, true)
    OrdinaryDiffEqDefault.step!(sim.integ, dt, true)
    return sim
end

"Current state as a NamedTuple in ArduPilot conventions."
function sim_state(sim::QuadSim)
    w, acc, r, v, q, rpy, vb, ib = sim.getstate(sim.integ)
    qn = sqrt(sum(abs2, q))
    return (timestamp = sim.integ.t,
            imu = (gyro = collect(Float64, w), accel_body = collect(Float64, acc)),
            position = collect(Float64, r),
            velocity = collect(Float64, v),
            quaternion = collect(Float64, q ./ qn),
            attitude = collect(Float64, rpy),
            battery = (voltage = Float64(vb), current = Float64(ib)))
end

"JSON reply for ArduPilot, framed by newlines."
sitl_json(sim::QuadSim) = "\n" * JSON.json(sim_state(sim)) * "\n"

"""
    run_ardupilot_bridge(; port = 9002, bind_ip = ip"0.0.0.0", max_frames = typemax(Int),
                           status_interval = 5.0, kwargs...)

Run the lock-step JSON SITL physics backend. Start this first, then launch
ArduPilot with `--model JSON:<ip of this machine>`. Keyword arguments other
than the ones listed are passed to [`build_quad_sim`](@ref) (e.g. `wind_ned`,
`yaw0`, `alg`). PWM channels 1..4 drive motors 1..4 (ArduPilot Quad-X order).

Frame handling: a repeated `frame_count` is answered with the previous reply
without stepping; a decreasing `frame_count` (SITL restart) resets the vehicle.
"""
function run_ardupilot_bridge(; port::Integer = 9002, bind_ip = ip"0.0.0.0",
                              max_frames::Integer = typemax(Int), status_interval::Real = 5.0,
                              sim::Union{Nothing, QuadSim} = nothing, kwargs...)
    @info "Compiling quadcopter model (first run can take a few minutes)..."
    sim === nothing && (sim = build_quad_sim(; kwargs...))
    sock = UDPSocket()
    bind(sock, bind_ip, port; reuseaddr = true) || error("Could not bind UDP $(bind_ip):$(port)")
    @info "ArduPilot JSON backend listening on UDP $(bind_ip):$(port)"
    last_count = -1
    last_reply = sitl_json(sim)
    nframes = 0
    next_status = status_interval
    connected = false
    try
        while nframes < max_frames
            addr, data = recvfrom(sock)
            frame = parse_sitl_frame(data)
            frame === nothing && continue
            if !connected
                @info "SITL connected from $(addr.host):$(addr.port), frame rate $(frame.frame_rate) Hz, $(length(frame.pwm)) channels"
                connected = true
            end
            if frame.frame_count < last_count
                reset_sim!(sim)
                next_status = status_interval
                @info "SITL restart detected (frame_count $(frame.frame_count) < $(last_count)): vehicle reset"
            elseif frame.frame_count == last_count
                send(sock, addr.host, addr.port, last_reply)
                continue
            elseif last_count >= 0 && frame.frame_count > last_count + 1
                @warn "Lost $(frame.frame_count - last_count - 1) SITL frame(s)"
            end
            dt = 1.0 / (frame.frame_rate > 0 ? frame.frame_rate : 400)
            step_sim!(sim, frame.pwm, dt)
            last_reply = sitl_json(sim)
            send(sock, addr.host, addr.port, last_reply)
            last_count = frame.frame_count
            nframes += 1
            if sim.integ.t >= next_status
                s = sim_state(sim)
                @info "t=$(round(s.timestamp, digits=1)) s  alt=$(round(-s.position[3], digits=2)) m  " *
                      "pwm=$(Int.(frame.pwm[1:4]))  batt=$(round(s.battery.voltage, digits=2)) V / $(round(s.battery.current, digits=1)) A"
                next_status += status_interval
            end
        end
    finally
        close(sock)
    end
    return sim
end
