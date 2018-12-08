#!/usr/bin/env julia

using RigidBodyDynamics
using StaticArrays
using Random

include("models.jl")

function julia_test(model)
    robot = parse_urdf(Float64, PATH * "$model.urdf")
    if model != "lwr" && model != "tiago"
        for joint in out_joints(root_body(robot), robot)
            floatingjoint = Joint(string(joint), frame_before(joint), frame_after(joint), QuaternionFloating{Float64}())
            replace_joint!(robot, joint, floatingjoint)
        end
    end

    println("Julia Test")
    println("  model: ", model)
    println("  nq: ", num_positions(robot))
    println("  nv: ", num_velocities(robot))

    state = MechanismState(robot)

    M = mass_matrix(state)
    println("FD: M after CRBA: ", M)

    tau = inverse_dynamics(state, state.v)
    println("ID: tau after RNEA: ", transpose(tau))
end

for model in MODELS
    julia_test(model)
end
