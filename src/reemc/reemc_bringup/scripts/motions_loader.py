#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2014 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ISC BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
# OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Authors:
# * Sammy Pfeiffer
'''
@author: Sammy Pfeiffer

Script that tries to load initially private motions
(in the case of working in PAL premises)
and if the package is not found loads the default
public ones
'''

from rospkg import RosPack, ResourceNotFound
from rosparam import upload_params
from yaml import load
import sys

PRIVATE_PKG_NAME = 'reemc_motions_proprietary'
PUBLIC_PKG_NAME = 'reemc_bringup'

PRIVATE_CONFIG_YAML = 'reemc_motions.yaml'
PUBLIC_CONFIG_YAML = 'reemc_motions.yaml'

def load_params_from_yaml(complete_file_path):
    '''Gets the params from a yaml file given the complete path of the
    file and uploads those to the param server.
    This is convenience function as rosparam.load_file does not
    give this functionality as found here:
    http://answers.ros.org/question/169866/load-yaml-with-code/'''
    f = open(complete_file_path, 'r')
    yamlfile = load(f)
    f.close()
    upload_params('/', yamlfile )

if __name__ == '__main__':
    # We take 'full' as the original robot model
    robot_name = 'full'
    # If the robot arg was set, we get the motions file related to that model
    for arg in sys.argv[1:]:
        if 'robot=' in arg:
            robot_name = arg.replace('robot=', '')
    if robot_name != 'full':
        PRIVATE_CONFIG_YAML = 'reemc_' + robot_name + '_motions.yaml'
        PUBLIC_CONFIG_YAML = 'reemc_' + robot_name + '_motions.yaml'

    rp = RosPack()
    print "Loading public motions from: " + PUBLIC_PKG_NAME
    pub_pkg_path = rp.get_path(PUBLIC_PKG_NAME)
    pub_config_yaml = PUBLIC_CONFIG_YAML
    pub_full_path = pub_pkg_path + '/config/' + pub_config_yaml
    try:
        load_params_from_yaml(pub_full_path)
    except IOError:
        # We should load at least some movements, so we hardcode here the really default file
        print str(pub_full_path) + " not found! Falling back to " + pub_pkg_path + '/config/reemc_motions.yaml'
        load_params_from_yaml(pub_pkg_path + '/config/reemc_motions.yaml')

    print "Trying to find private package: " + PRIVATE_PKG_NAME
    try:
        pkg_path = rp.get_path(PRIVATE_PKG_NAME)
        config_yaml = PRIVATE_CONFIG_YAML
        full_path = pkg_path + '/config/' + config_yaml
        print "Loading params from: " +  full_path
        load_params_from_yaml(full_path)
    except ResourceNotFound:
        print "Not found, only using public motions."
    except IOError:
        print "Package found but " + config_yaml + " not found, only using public motions."

    print "Finished."
    