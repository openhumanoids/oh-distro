close all 

mode =1
if (mode==0)
  fpath = '/home/mfallon/drc/software/perception/stereo-bm'
  fname = '/disparity_lcm_19029000.png'
elseif (mode==1)
  fpath = '/home/mfallon/drc/software/perception/stereo-bm'
  fname = '/disparity_lcm_32f_22928000.png'
else
  fpath = '/home/mfallon/drc/software/perception/registeration'
  fname = 'crl_disparity_lcm_1360258422626224.png'
end
  
ls(fpath)
x=imread([fpath '/' fname ]);


imagesc(x)


%imagesc(x, [0 256])
colorbar