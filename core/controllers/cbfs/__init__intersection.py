import numpy as np
from .cbf import Cbf
from .symbolic_cbfs.collision_avoidance_2d import h_ca, dhdx_ca, d2hdx2_ca, h_rpca, dhdx_rpca, d2hdx2_rpca
# from .symbolic_cbfs.road_safety import h0_road, h_road, dhdx_road, d2hdx2_road
from .symbolic_cbfs.radial_safety import h_radial, dhdx_radial, d2hdx2_radial
from .symbolic_cbfs.speed_safety import h_speed, dhdx_speed, d2hdx2_speed
# from .symbolic_cbfs.stochastic_ho_cbf import C as h_ho, dCdx as dhdx_ho, d2Cdx2 as d2hdx2_ho


def linear_class_k(k):

    def alpha(h):
        return k*h

    return alpha


# Define linear class k weights
k_default = 5.0
k_collision = 1.0

# Define cbf lists
# cbfs_individual = [
#     Cbf(h_road, dhdx_road, d2hdx2_road, linear_class_k(k_road), h0_road),  # Highway Safety (hs)
# ]
cbfs_individual = [
    Cbf(h_radial, dhdx_radial, d2hdx2_radial, linear_class_k(k_default), h_radial),
    Cbf(h_speed, dhdx_speed, d2hdx2_speed, linear_class_k(k_default), h_speed)
]
cbfs_pairwise = [
    Cbf(h_rpca, dhdx_rpca, d2hdx2_rpca, linear_class_k(k_collision), h_ca),  # Collision Avoidance (ca)
]  # RV-CBF
cbfs_pairwise_nn = []
# cbfs_pairwise_nn = [Cbf(), Cbf(h_ho, dhdx_ho, d2hdx2_ho, linear_class_k(k))]

cbf0 = np.zeros((len(cbfs_individual) + len(cbfs_pairwise), ))
