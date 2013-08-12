#!/usr/bin/env ruby
# the gains file is mindless, so this script generates it for you
ambi_joints = []
4.times do |f_idx|
  3.times do |j_idx|
    ambi_joints += ["f#{f_idx}_j#{j_idx}"]
  end
end
joints = []
["left", "right"].each do |prefix|
  ambi_joints.each do |joint|
    joints += ["#{prefix}_#{joint}"]
  end
end
File.open("sandia_hand_gazebo_gains.yaml","w") do |f|
  f.puts "gains:"
  joints.each do |joint|
    f.puts "  #{joint}: {p: 2.0, d: 0.5, i: 0.0, i_clamp: 0.0}"
  end
end
