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


from dynamic_reconfigure.parameter_generator_catkin import str_t
from dynamic_reconfigure.parameter_generator_catkin import double_t
from dynamic_reconfigure.parameter_generator_catkin import bool_t

# set up parameters that we care about
PACKAGE = 'pcl_ros'


def add_common_parameters(gen):
    # def add (self, name, paramtype, level, description, default = None, min = None,
    # max = None, edit_method = ""):
    gen.add("filter_field_name", str_t, 0, "The field name used for filtering", "z")
    gen.add("filter_limit_min", double_t, 0,
            "The minimum allowed field value a point will be considered from",
            0.0, -100000.0, 100000.0)
    gen.add("filter_limit_max", double_t, 0,
            "The maximum allowed field value a point will be considered from",
            1.0, -100000.0, 100000.0)
    gen.add("filter_limit_negative", bool_t, 0,
            ("Set to true if we want to return the data outside "
             "[filter_limit_min; filter_limit_max]."),
            False)
    gen.add("keep_organized", bool_t, 0,
            ("Set whether the filtered points should be kept and set to NaN, "
             "or removed from the PointCloud, thus potentially breaking its organized structure."),
            False)
    gen.add("input_frame", str_t, 0,
            ("The input TF frame the data should be transformed into before processing, "
             "if input.header.frame_id is different."),
            "")
    gen.add("output_frame", str_t, 0,
            ("The output TF frame the data should be transformed into after processing, "
             "if input.header.frame_id is different."),
            "")
