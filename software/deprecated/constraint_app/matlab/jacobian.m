function J = jacobian(ptr, x, observationIds)

    J = cam_getJacobian(ptr, x, observationIds, 1)