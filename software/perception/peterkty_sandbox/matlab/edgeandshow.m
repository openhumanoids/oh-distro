function edgeandshow(IM, thresh)
  
  %IM = imresize(IM, 0.5);
  IM = imresize(IM, 1);
  IMGray = rgb2gray(IM);
  
  %g = fspecial('gaussian');
  %IMGray = imfilter(IMGray,g,'same');
  %thresh = 0.001;
  [E thresh] = edge(IMGray, 'log', thresh);
  fprintf('thresh=%f\n', thresh);
  
  
  se = strel('disk',2);
  edgeshow = E;
  edgeshow = imdilate(edgeshow,se);
  
  edgeshow = repmat(uint8(edgeshow),[1,1,3]);
  edgeshow(:,:,3) = 0;
  edgeshow(:,:,1) = 0;
  
  figure(2);
  imshow(IM);
  
  %IM = max(IM, edgeshow*255);
  figure(1);
  imshow(E);
end