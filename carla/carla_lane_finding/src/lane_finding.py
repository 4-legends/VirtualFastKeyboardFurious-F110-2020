# -*- encoding: utf-8 -*-

import cv2
import numpy as np
import math
from sklearn.cluster import MeanShift

class LaneFinder():
    def __init__(self):
        self.vertices = np.array([[(120,600), (331, 360), (480, 360),(650,600)]], dtype=np.int32)
        self.ox = 400
        self.oy = 300
        self.fx = 335.63985247
        self.fy =  335.63985247
        # self.kmeans = MeanShift() #n_clusters=2, random_state=0)
        self.ms = MeanShift() #n_clusters=2, random_state=0)

    def processor(self, img, depth):

        #add noise (blur) to image because of motion
        image=cv2.GaussianBlur(img, (5, 5), 0)
        #grayscale
        out1= cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        imshape = out1.shape
        #edge detector
        out2= cv2.Canny(out1, 70, 150)  
        #region of interest
        out3=self.region_of_interest(out2)
        
        #hough transform
        rho = 1 # distance resolution in pixels of the Hough grid
        theta = np.pi/180 # angular resolution in radians of the Hough grid
        threshold = 20     # minimum number of votes (intersections in Hough grid cell)
        min_line_len = 10  #minimum number of pixels making up a line
        max_line_gap = 3    # maximum gap in pixels between connectable line segments
        color=[0, 0, 255] #red bgr format

        line_image=self.hough_lines(out3, rho, theta, threshold, min_line_len, max_line_gap,color)
        
        #weighted image 
        out=self.weighted_img(line_image, image, alpha=0.8, beta=1., gamma=0.)
        # cv2.imwrite(dir_parent + 'detections/%s'%(imgs[i]), out)
        cv2.imwrite('out.jpg', out)
        ref_points = self.determine_reference_points(line_image)
        if ref_points[0]==None:
            return None, None, False
        else: 
            carla_ref_points = self.transform_with_depth(depth, ref_points)
            return out, carla_ref_points, True


    def perform_clustering(self, points):
        self.ms.fit(points.T)
        coord = self.ms.cluster_centers_[-1].astype(np.int64)[0]
        return coord
    def determine_reference_points(self,  line_image):
        first_right, second_right, center= None, None, None
        for i in range(580, 0, -1):
            idxs = np.array(np.where(np.any(line_image[i]>0, axis=1)))
            print (idxs)
            idxs = idxs[idxs>400]
            if idxs.size !=0 :
                               
                
                # if idxs.shape[0]==1:
                #     idxs = self.perform_clustering(idxs)
                first_right = [idxs[0], i]
                center =  [idxs[0] - 180, i]
                new_start_idx = i - 50
                break
                
        for i in range(new_start_idx, 0, -1):
            idxs = np.array(np.where(np.any(line_image[i]>0, axis=1)))
            idxs = idxs[idxs>400]

            if idxs.size !=0 :
                # idxs = idxs[idxs>400]
                print ('idxs', idxs, idxs.shape)
                
                # if idxs.shape[0]==1:
                #     idxs = self.perform_clustering(idxs)
                # print (idxs[0])
                second_right = [idxs[0], i]
                break
        
        return first_right, second_right, center


    def transform_with_depth(self, depth, ref_points):
        t_ref_points = []
        for i in range(len(ref_points)):
            # print (depth[ref_points[i][::-1]])
            x= ref_points[i][0] 
            y= ref_points[i][1] 
            print ('yolo2', x,y, depth[y,x])
            z = np.log(depth[y,x])
            x= ref_points[i][0] 
            y= ref_points[i][1] 
            
            realworldz = z
            realworldx = (x - self.ox) * z * (1/self.fx)
            realworldy = (y - self.oy) * z * (1/self.fy)
            t_ref_points.append([realworldx, realworldy, realworldz])
        return t_ref_points
  
    def region_of_interest(self, img):
        """
        Applies an image mask.
        
        Only keeps the region of the image defined by the polygon
        formed from `vertices`. The rest of the image is set to black.
        """
        #defining a blank mask to start with
        mask = np.zeros_like(img)   
        
        #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
        if len(img.shape) > 2:
            channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255
            
        #filling pixels inside the polygon defined by "vertices" with the fill color    
        cv2.fillPoly(mask, self.vertices, ignore_mask_color)
        
        #returning the image only where mask pixels are nonzero
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image


    def draw_lines(self, img, lines, color, thickness=4):
        """
        NOTE: this is the function you might want to use as a starting point once you want to 
        average/extrapolate the line segments you detect to map out the full
        extent of the lane (going from the result shown in raw-lines-example.mp4
        to that shown in P1_example.mp4).  
        
        Think about things like separating line segments by their 
        slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
        line vs. the right line.  Then, you can average the position of each of 
        the lines and extrapolate to the top and bottom of the lane.
        
        This function draws `lines` with `color` and `thickness`.    
        Lines are drawn on the image inplace (mutates the image).
        If you want to make the lines semi-transparent, think about combining
        this function with the weighted_img() function below
        """
        #left_lines=[]
        #right_lines=[]
        right_slope=0
        right_intercept=0
        left_slope=0
        left_intercept=0
        n=1
        right_line = np.array([], dtype=np.int64).reshape(0,4)
        left_line= np.array([], dtype=np.int64).reshape(0,4)
        #avg of all (rho, theta)
        for line in lines:
            for x1,y1,x2,y2 in line:
    #             cv2.line(img, (x1,y1),(x2,y2), color, thickness)
                slope=(y2-y1)/(x2-x1)
                if(slope>0):
                    right_slope=right_slope+slope
                    right_intercept=right_intercept+(y1-slope*x1)
                    right_line=np.vstack((right_line,np.array([x1,y1,x2,y2])))
                else:
                    left_slope=left_slope+slope
                    left_intercept=left_intercept+(y1-slope*x1)
                    left_line=np.vstack((left_line,np.array([x1,y1,x2,y2])))
            n=n+1
        right_slope=right_slope/n
        left_slope=left_slope/n
        #to not to consider outlier we introduce threshold not to go far away from average 
        n=0
        for x1,y1,x2,y2 in left_line:
            slope=(y2-y1)/(x2-x1)
            #not too far from average slope 
            if((abs(slope-left_slope)>0.1)):
                #delete those values
                np.delete(left_line, n, 0)
            else:
                n=n+1
        n=0
        for x1,y1,x2,y2 in right_line:
            slope=(y2-y1)/(x2-x1)
            if(abs(slope-right_slope)>0.1):
                #delete those values
                np.delete(right_line, n, 0)
            else:
                n=n+1            
                
        left_max=np.amax(left_line, axis=0)
        left_min=np.amin(left_line, axis=0)
        right_max=np.amax(right_line, axis=0)
        right_min=np.amin(right_line, axis=0)
        y2l=left_min[3]
        x2l=left_max[2]
        y1l=left_max[1]
        x1l=left_min[0]
        
        y2r=right_max[3]
        x2r=right_max[2]
        y1r=right_min[1]
        x1r=right_min[0]
        
        cv2.line(img, (x1r,y1r),(x2r,y2r), color, thickness)
        cv2.line(img, (x1l,y1l),(x2l,y2l), color, thickness)

    def hough_lines(self, img, rho, theta, threshold, min_line_len, max_line_gap,color):
        """
        `img` should be the output of a Canny transform.
            
        Returns an image with hough lines drawn.
        """
        lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        self.draw_lines(line_img, lines,color)
        return line_img

    # Python 3 has support for cool math symbols.

    def weighted_img(self, img, initial_img, alpha=0.8, beta=1., gamma=0.):
        """
        `img` is the output of the hough_lines(), An image with lines drawn on it.
        Should be a blank image (all black) with lines drawn on it.
        
        `initial_img` should be the image before any processing.
        
        The result image is computed as follows:
        
        initial_img * α + img * β + λ
        NOTE: initial_img and img must be the same shape!
        """
        return cv2.addWeighted(initial_img, alpha, img, beta, gamma)
