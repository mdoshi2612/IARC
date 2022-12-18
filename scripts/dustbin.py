def tracker(camera):
    while True:
        camera.click_depth_image()
        camera.click_rgb_image()
        bgr = camera.img_bgr
        depth = camera.img_depth
        img_mask = dect.color_threshold(bgr)
        thresh, close, image, center = dect.final_detection(bgr.copy(), img_mask)
        cv2.imshow('Result', thresh)
        cv2.imshow('Result1', close)
        cv2.imshow('Result2', image)
        if len(center) > 0:
            pc, pc_sense = plane.point_cloud(np.array(center), depth)
            sense = line.best_line(pc_sense)
            sense = -1*sense[0]/sense[1]
            m = plane.face_vector(pc)
            a, b, c, d = m
            angle = trans.angle_with_z([a, b, c])
            xyz = dect.loc(depth, center)
            #print(xyz)
            print(angle)
            print(sense)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()