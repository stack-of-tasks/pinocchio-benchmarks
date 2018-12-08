options = Base.JLOptions()

@assert VERSION >= v"1.0.2"
@assert options.can_inline == 1
@assert options.check_bounds == 2
@assert options.opt_level == 3

using RigidBodyDynamics
using Random
using LinearAlgebra

include("models.jl")

function write_times(log_filename, times)
    file = open(log_filename, "w") do io
        println.(Ref(io), times[2 : end]) # skip the first sample, which includes compilation time
    end
    return nothing
end

function benchmark_julia_rnea(model, log_filename)
    robot = parse_urdf(PATH * "$model.urdf", floating=model != "lwr")
    state = MechanismState(robot)
    result = DynamicsResult(robot)
    qs = [similar(configuration(state)) for _ = 1 : NBT]
    vs = [similar(velocity(state)) for _ = 1 : NBT]
    v̇s = [similar(velocity(state)) for _ = 1 : NBT]
    τ = similar(velocity(state))
    times = Vector{Int}(undef, NBT)
    Random.seed!(1)
    for i = 1 : NBT
        rand!(state)
        qs[i] .= configuration(state)
        vs[i] .= velocity(state)
        rand!(v̇s[i])
    end
    run_rnea_benchmark(state, qs, vs, v̇s, τ, result.jointwrenches, result.accelerations, times)
    write_times(log_filename, times)
    return nothing
end

function run_rnea_benchmark(state, qs, vs, v̇s, τ, jointwrenches, accelerations, times)
    # Do garbage collection of previously allocated objects now, rather than randomly
    # in the middle of the benchmark.
    # If garbage is generated during the benchmark run, so be it (it shouldn't in this case),
    # but time spent doing garbage collection of objects allocated outside this loop shouldn't be included.
    GC.gc()
    for i in eachindex(qs)
        debut = time_ns()
        copyto!(configuration(state), qs[i])
        copyto!(velocity(state), vs[i])
        setdirty!(state)
        v̇ = v̇s[i]
        inverse_dynamics!(τ, jointwrenches, accelerations, state, v̇)       
        fin = time_ns()
        times[i] = Int(fin - debut)
    end
    return nothing
end

function benchmark_julia_crba(model, log_filename)
    robot = parse_urdf(PATH * "$model.urdf", floating=model != "lwr")
    state = MechanismState(robot)
    qs = [similar(configuration(state)) for _ = 1 : NBT]
    times = Vector{Int}(undef, NBT)
    nv = num_velocities(robot)
    M = Symmetric(Matrix{Float64}(undef, nv, nv), :L)
    Random.seed!(1)
    for i = 1 : NBT
        rand!(state)
        qs[i] .= configuration(state)
    end
    run_crba_benchmark(state, qs, M, times)
    write_times(log_filename, times)
    return nothing
end

function run_crba_benchmark(state, qs, M, times)
    # Do garbage collection of previously allocated objects now, rather than randomly
    # in the middle of the benchmark.
    # If garbage is generated during the benchmark run, so be it (it shouldn't in this case),
    # but time spent doing garbage collection of objects allocated outside this loop shouldn't be included.
    for i in eachindex(qs)
        debut = time_ns()
        copyto!(configuration(state), qs[i])
        setdirty!(state)
        mass_matrix!(M, state)
        fin = time_ns()
        times[i] = Int(fin - debut)
    end
end

for model in MODELS
    benchmark_julia_rnea(model, get_log_filename("Julia", "ID", model))
    benchmark_julia_crba(model, get_log_filename("Julia", "CRBA", model))
end

