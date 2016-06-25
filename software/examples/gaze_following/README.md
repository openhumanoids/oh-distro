# Gaze following demo

To start:

1. Run a bot-procman-sheriff with the pmd file: ``bot-procman-sheriff gaze_following.pmd``
2. Open director and visualize the april_tag_to_car_beam frame, then set the imageWidget to display KINECT_RGB
3. Make sure the Multisense driver is started, or alternatively use ``oh-webcam-driver -m``
4. Use the start script in the pmd to start the required processes

Alternatively:

1. Run ``drc-car-tags -p -q -c KINECT_RGB``
2. Run ``oh-gaze-following-demo``
