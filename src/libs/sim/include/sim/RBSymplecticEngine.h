//
// Created by Dongho Kang on 25.04.22.
//

#ifndef A5_RBSYMPLECTICENGINE_H
#define A5_RBSYMPLECTICENGINE_H

#include "sim/RBEngine.h"

namespace crl {

/**
 * simulation world implemented with symplectic integration scheme.
 */
class RBSymplecticEngine : public RBEngine {
public:
    RBSymplecticEngine() : RBEngine() {}

    ~RBSymplecticEngine() override = default;

    void step(double dt) override {
        // external force and torque
        updateForceForGravity();
        updateForceAndTorqueForSprings();

        // update states of rbs (integration)
        for (uint i = 0; i < rbs.size(); i++) {
            RB *rb = rbs[i];
            // retrieve saved force and tau
            V3D f = f_ext[i];
            V3D tau = tau_ext[i];

            // TODO: Ex.3 Stable Simulation
            // why our simulation is blown up? let's make it more stable using
            // symplectic Euler integration!

            rb->state.velocity += dt * f / rb->rbProps.mass;         // TODO: change this!
            Matrix3x3 R = rb->state.orientation.normalized().toRotationMatrix();
            Matrix3x3 I_b = rb->rbProps.MOI_local;
            Matrix3x3 I = R * I_b * R.transpose();
            V3D w_i = rb->state.angularVelocity;
            rb->state.angularVelocity += dt * I.inverse() * (tau - w_i.cross(V3D(I * w_i)));  // TODO: change this!

            if (simulateCollisions && rb->rbProps.collision)
                updateVelocityAfterCollision(rb);

            // TODO: Ex.3 Stable Simulation
            // implement forward (explicit) Euler integration scheme for computing pose.
            rb->state.pos = rb->state.pos + rb->state.velocity * dt;  // TODO: change this!
            rb->state.orientation = updateRotationGivenAngularVelocity(
                rb->state.orientation, rb->state.angularVelocity, dt);
        }

        // clean up
        for (uint i = 0; i < f_ext.size(); i++) {
            f_ext[i] = V3D();
            tau_ext[i] = V3D();
        }
    }
};

}  // namespace crl

#endif  //A5_RBSYMPLECTICENGINE_H
