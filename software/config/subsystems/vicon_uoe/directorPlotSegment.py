def showUoeViconMarkers():
	# Used for experiments - 3 May 2016
	# from pelvis_val.vsk
	m1 = np.array([42.430198669433594, 0.38291072845458984, -75.745498657226562]) * 0.001
	m2 = np.array([36.803276062011719, 0.23376750946044922, -16.981033325195312]) * 0.001
	m3 = np.array([-24.306129455566406, -119.755126953125, 47.446334838867188]) * 0.001
	m4 = np.array([-54.927345275878906, 119.13845062255859, 45.280197143554688]) * 0.001

	# measurment info from jlack:
	point1 = [0.1099,0,-0.2015]
	point2 = [0.1099,0,-0.14053]
	point3 = [0.0512,-0.1194,-0.0717]
	point4 = [0.0512,0.1194,-0.0717]
	# 5cm and 2.5cm extensions with 7.5mm diameter marker dots
	point1[0] += 0.0575
	point2[0] += 0.0575
	point3[0] += 0.0575
	point4[0] += 0.0325

	d2 = DebugData()
	d2.addSphere(m1, radius=0.01)
	d2.addSphere(m2, radius=0.01)
	d2.addSphere(m3, radius=0.01)
	d2.addSphere(m4, radius=0.01)

	d = DebugData()
	d.addSphere(point1, radius=0.01)
	d.addSphere(point2, radius=0.01)
	d.addSphere(point3, radius=0.01)
	d.addSphere(point4, radius=0.01)

	worldToPelvis = transformUtils.copyFrame( getLinkFrame( 'pelvis') )

	# hand made alignment
	pos = [-0.143, -0.002, 0.1065]
	rpy = [0, -6.*np.pi/180.0, 1.0*np.pi/180.0]

	quat = transformUtils.rollPitchYawToQuaternion(rpy)
	viconToPelvis = transformUtils.transformFromPose(pos, quat)


	pd = segmentation.transformPolyData(d.getPolyData(), worldToPelvis)
	obj = updatePolyData(pd, 'nasa model', visible=True, color=[1,0,0])



	worldToVicon =  transformUtils.copyFrame(worldToPelvis)
	worldToVicon.PreMultiply()
	worldToVicon.Concatenate( viconToPelvis.GetLinearInverse() )

	vis.showFrame(worldToVicon,'vicon markers frame')

	pd2 = segmentation.transformPolyData(d2.getPolyData(), worldToVicon)
	obj = updatePolyData(pd2, 'vicon markers', visible=True, color=[0,1,0])
