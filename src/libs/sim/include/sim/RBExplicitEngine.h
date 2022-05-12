//
// Created by Dongho Kang on 25.04.22.
//

#ifndef A5_RBEXPLICITENGINE_H
#define A5_RBEXPLICITENGINE_H

#include "sim/RBEngine.h"

namespace crl {

/**
 * simulation world implemented with explicit integration scheme.
 */
class RBExplicitEngine : public RBEngine {
public:
    RBExplicitEngine() : RBEngine() {}

    ~RBExplicitEngine() override = default;

    void step(double dt) override {
        // update external force and torque
        updateForceForGravity();
        updateForceAndTorqueForSprings();

        // update states of rbs (integration)
        for (uint i = 0; i < rbs.size(); i++) {
            RB *rb = rbs[i];
            // retrieve saved force and tau
            V3D f = f_ext[i];
            V3D tau = tau_ext[i];

            // TODO: Done! Ex.1 Numerical Integration
            // implement forward (explicit) Euler integration scheme for computing velocity.
            //
            // Hint:
            // - complete the function,
            // Quaternion updateRotationGivenAngularVelocity(const Quaternion &q, const V3D &angularVelocity, double dt)
            // in src/libs/sim/include/sim/RBEngine.h and use it for updating orientation of rigidbody.
            // - recall, you need to compute 3x3 moment of inertia matrix expressed in world frame.
            V3D v_i = rb->state.velocity;
            rb->state.velocity += dt * f / rb->rbProps.mass;         // TODO: Done! change this!
            Matrix3x3 R = rb->state.orientation.normalized().toRotationMatrix();
            Matrix3x3 I_d = rb->rbProps.MOI_local;
            Matrix3x3 I = R * I_d * R.transpose();
            V3D w_i = rb->state.angularVelocity;
            rb->state.angularVelocity += dt * I.inverse() * (tau - w_i.cross(V3D(I * w_i)));  // TODO: Done! change this!

            if (simulateCollisions && rb->rbProps.collision)
                updateVelocityAfterCollision(rb);

            // TODO: Done! Ex.1 Numerical Integration
            // implement forward (explicit) Euler integration scheme for computing pose.
            rb->state.pos = rb->state.pos + v_i * dt;  // TODO: Done! change this!
            rb->state.orientation = updateRotationGivenAngularVelocity(
                rb->state.orientation, w_i, dt);
        }

        // clean up
        for (uint i = 0; i < f_ext.size(); i++) {
            f_ext[i] = V3D();
            tau_ext[i] = V3D();
        }
    }
};

}  // namespace crl

#endif  //A5_RBEXPLICITENGINE_H
