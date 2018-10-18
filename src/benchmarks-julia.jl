#!/usr/bin/env julia

using RigidBodyDynamics
using StaticArrays

include("models.jl")

function benchmark_julia_rnea(model, log_filename)
    robot = parse_urdf(Float64, PATH * "$model.urdf")
    if model != "lwr"
        for joint in out_joints(root_body(robot), robot)
            floatingjoint = Joint(string(joint), frame_before(joint), frame_after(joint), QuaternionFloating{Float64}())
            replace_joint!(robot, joint, floatingjoint)
        end
    end

    states = Array{MechanismState}(undef, NBT)
    for i = 1:NBT
        states[i] = MechanismState(robot)
        rand!(states[i])
    end

    file = open(log_filename, "w")
    for i = 1:NBT
        debut = time_ns()
        inverse_dynamics(states[i], states[i].v)
        fin = time_ns()
        write(file, Int(fin - debut))
    end
    close(file)
end

function benchmark_julia_crba(model, log_filename)
    robot = parse_urdf(Float64, PATH * "$model.urdf")
    if model != "lwr"
        for joint in out_joints(root_body(robot), robot)
            floatingjoint = Joint(string(joint), frame_before(joint), frame_after(joint), QuaternionFloating{Float64}())
            replace_joint!(robot, joint, floatingjoint)
        end
    end

    states = Array{MechanismState}(undef, NBT)
    for i = 1:NBT
        states[i] = MechanismState(robot)
        rand!(states[i])
    end


    file = open(log_filename, "w")
    for i = 1:NBT
        debut = time_ns()
        mass_matrix(states[i])
        fin = time_ns()
        write(file, Int(fin - debut))
    end
    close(file)
end

for model in MODELS
    benchmark_julia_rnea(model, get_log_filename("Julia", "ID", model))
    benchmark_julia_crba(model, get_log_filename("Julia", "FD", model))
end
