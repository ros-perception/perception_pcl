#! /usr/bin/env python

# Copyright 2009 Willow Garage, Inc
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Willow Garage, Inc nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from dynamic_reconfigure.parameter_generator_catkin import int_t
from dynamic_reconfigure.parameter_generator_catkin import double_t
from dynamic_reconfigure.parameter_generator_catkin import bool_t
from dynamic_reconfigure.parameter_generator_catkin import str_t

# set up parameters that we care about
PACKAGE = 'pcl_ros'


def add_common_parameters(gen):
    # add(self, name, paramtype, level, description, default = None, min = None,
    # max = None, edit_method = "")
    gen.add("max_iterations", int_t, 0,
            "The maximum number of iterations the algorithm will run for",
            50, 0, 100000)
    gen.add("probability", double_t, 0,
            "The desired probability of choosing at least one sample free from outliers",
            0.99, 0.5, 0.99)
    gen.add("distance_threshold", double_t, 0,
            "The distance to model threshold",
            0.02, 0, 1.0)
    gen.add("optimize_coefficients", bool_t, 0,
            "Model coefficient refinement",
            True)
    gen.add("radius_min", double_t, 0,
            "The minimum allowed model radius (where applicable)",
            0.0, 0, 1.0)
    gen.add("radius_max", double_t, 0,
            "The maximum allowed model radius (where applicable)",
            0.05, 0, 1.0)
    gen.add("eps_angle", double_t, 0,
            ("The maximum allowed difference between the model normal "
             "and the given axis in radians."),
            0.17, 0.0, 1.5707)
    gen.add("min_inliers", int_t, 0,
            "The minimum number of inliers a model must have in order to be considered valid.",
            0, 0, 100000)
    gen.add("input_frame", str_t, 0,
            ("The input TF frame the data should be transformed into, "
             "if input.header.frame_id is different."),
            "")
    gen.add("output_frame", str_t, 0,
            ("The output TF frame the data should be transformed into, "
             "if input.header.frame_id is different."),
            "")
