#!/usr/bin/env python
# Software License Agreement (BSD License)                                                                                           
#                                                                                                                                    
# Copyright (c) 2011, Willow Garage, Inc.                                                                                            
# All rights reserved.                                                                                                               
#                                                                                                                                    
# Redistribution and use in source and binary forms, with or without                                                                 
# modification, are permitted provided that the following conditions                                                                 
# are met:                                                                                                                           
#                                                                                                                                    
#  * Redistributions of source code must retain the above copyright                                                                  
#    notice, this list of conditions and the following disclaimer.                                                                   
#  * Redistributions in binary form must reproduce the above                                                                         
#    copyright notice, this list of conditions and the following                                                                     
#    disclaimer in the documentation and/or other materials provided                                                                 
#    with the distribution.                                                                                                          
#  * Neither the name of Willow Garage, Inc. nor the names of its                                                                    
#    contributors may be used to endorse or promote products derived                                                                 
#    from this software without specific prior written permission.                                                                   
#                                                                                                                                    
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS                                                                
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT                                                                  
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS                                                                  
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE                                                                     
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,                                                                
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,                                                               
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                                                                   
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER                                                                   
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT                                                                 
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN                                                                  
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE                                                                    
# POSSIBILITY OF SUCH DAMAGE.      


import os
import subprocess

from optparse import OptionParser

parser = OptionParser(usage='add_debs repo_path distro directory', prog='add_debs')

options, args = parser.parse_args()


if len(args) != 3:
    parser.error("Please enter a repo and a path to deb files")


repo = args[0]
distro = args[1]
directory = args[2]

repo_contents = os.listdir(repo)
expected = ['pool', 'db', 'dists', 'pool']
missing = [e for e in expected if e not in repo_contents]
if len(missing) > 0:
    parser.error("repo [%s] doesn't look like a repo.  it's missing %s"%(repo, missing))


valid_distros = ['lucid', 'maverick']
if distro not in valid_distros:
    parser.error("distro [%s] is not a valid distro: %s"%(distro, valid_distros))

dir_contents = os.listdir(directory)
debs = [os.path.join(directory, d) for d in dir_contents if ".deb" in d]
if len(debs) == 0:
    parser.error("directory [%s] doesn't contain deb files"%directory)


for d in debs:
    print "pushing %s into %s %s"%(d, repo, distro)
    subprocess.check_call(('reprepro -V -b %s includedeb %s %s'%(repo, distro, d)).split())
