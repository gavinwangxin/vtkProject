"""
author: WANG Xin

Done:
project 3D model from multiple viewpoints into 2D images;
save ROI 2D image;

To do:
more input format ply...;
adaptive select ROI;

"""
import os
import sys

import numpy as np
import vtk
from vtk.util.numpy_support import vtk_to_numpy
from vtk.util import numpy_support
import cv2
print(cv2.__version__)
print(vtk.vtkVersion.GetVTKSourceVersion())


def rotation_matrix4x4(matrix):
    m = np.ones((4, 4))
    for i in range(4):
        for j in range(4):
            m[i, j] = matrix.GetElement(i, j)
    return m


class CameraProject:

    def __init__(self,filename,N):
        """
        max_camera_points:N
        filedir: filename
        """
        self.filename = filename
        self.num_point = N
        self.timer_count = 0

        self.reader = None
        self.ColorBackground= [0,0,0]
        self.viewpoint = None
        self.points = None

        self.timerId = None

        self.windowSize = 640
        self.actor = vtk.vtkActor()
        self.renderer = vtk.vtkRenderer()
        self.renWin = vtk.vtkRenderWindow()
        self.iren = vtk.vtkRenderWindowInteractor()
        self.renWin.SetSize(self.windowSize, self.windowSize)

    def read_file(self):
        """
        Read different format file
        :param filename:
        :return:
        """
        # obj
        # reader = vtk.vtkO
        try:
            self.reader = vtk.vtkOBJReader()
            self.reader.SetFileName(self.filename)
            self.reader.Update()
            ug = self.reader.GetOutput()
            vtk_points = ug.GetPoints()
            self.points = vtk_to_numpy(vtk_points.GetData())
            # print(points.shape)
        except IOError:
            print("cannnot read obj file.")
        return self.points

    # def toy_synthesize_model(self):
    #     """
    #     toy sphere
    #     :return:
    #     """
    #     self.shpereSource = vtk.vtkSphereSource()
    #     self.shpereSource.SetCenter(10,10,10)
    #     self.shpereSource.SetRadius(2.0)
    #     return self.shpereSource

    def generate_viewpoint(self, num_point):
        """
        n points distribute evenly on surface of unit sphere
        :param model:
        :param num_point:
        :return:
        """
        z = 2 * np.random.rand(num_point)-1
        t = 2 * np.pi * np.random.rand(num_point)
        x = np.sqrt(1 - z**2) * np.cos(t)
        y = np.sqrt(1 - z**2) * np.sin(t)
        self.viewpoint = np.asarray([x,y,z]).T
        return self.viewpoint

    def timer_callback(self,obj,event):
        """

        :param viewpoint:
        :return:
        """
        row, col = self.points.shape
        raw_points = np.ones((row,col+1))
        raw_points[:,:-1]=self.points
        print(raw_points[0,:])

        cam = self.renderer.GetActiveCamera()
        print(cam.GetPosition())

        near, far = cam.GetClippingRange()
        w, h = self.iren.GetSize()
        # print(w,h)
        # print(near,far)
        count = 0

        for view in self.viewpoint:

            cam.SetPosition(view[0],view[1],view[2])
            print(cam.GetPosition())
            K_c = rotation_matrix4x4(cam.GetProjectionTransformMatrix(w*1.0/h,near,far))
            Rt = rotation_matrix4x4(cam.GetModelViewTransformMatrix())
            # print(Rt)
            # point position in camera frame
            Pos_c = Rt.dot(raw_points.T)
            Pos_im = K_c.dot(Pos_c)
            print(Pos_im.shape)
            Pos_im = Pos_im/Pos_im[-1]*self.windowSize/2.0

            self.renWin.Render()

            #convert Pos_img to img
            # winToIm = vtk.vtkWindowToImageFilter()
            winToIm = vtk.vtkWindowToImageFilter()
            winToIm.SetInput(self.renWin)
            winToIm.Update()
            vtk_image = winToIm.GetOutput()
            # convert vtk_image to readable opencv file
            height, width, _ = vtk_image.GetDimensions()
            vtk_array = vtk_image.GetPointData().GetScalars()
            components = vtk_array.GetNumberOfComponents()
            arr = cv2.flip(numpy_support.vtk_to_numpy(vtk_array).reshape(height, width, components), 0)
            count+=1
            r=[200,200,240,240]
            arr1 = arr[r[0]:r[0]+r[2],r[1]:r[1]+r[3]]

            #save image
            outfile = '007/'+str('{:04d}'.format(count))+'.jpg'
            cv2.imwrite(outfile,arr1)


            cv2.imshow('image',arr1)
            cv2.waitKey(1)


        self.timer_count +=1








    def show_model(self):
        # self.renderer = vtk.vtkRenderer()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(self.reader.GetOutputPort())

        # self.actor = vtk.vtkActor()
        self.actor.SetMapper(mapper)
        # self.renWin = vtk.vtkRenderWindow()
        # self.iren = vtk.vtkRenderWindowInteractor()

        self.renderer.AddActor(self.actor)
        self.renderer.SetBackground(self.ColorBackground)
        self.renderer.ResetCamera()

        self.renWin.AddRenderer(self.renderer)
        self.renWin.SetSize(self.windowSize, self.windowSize)
        self.iren.SetRenderWindow(self.renWin)
        self.iren.Initialize()
        self.iren.AddObserver('TimerEvent', self.timer_callback)
        # TimerEvent write wrong
        self.timerId = self.iren.CreateRepeatingTimer(200)
        self.iren.Start()


    def run(self):
        """
        1. read file
        2. generate active camera viewpoint
        3. project model from viewpoint
        4. save image
        :return:
        """
        # dir = '007'
        #
        # os.mkdir(dir)

        # Read model file and show
        self.points = self.read_file()
        # sphere = self.toy_synthesize_model()
        #generate multiple viewpoint
        viewpoint = self.generate_viewpoint(self.num_point)
        self.show_model()

        # project model from viewpoint
        # self.project_model_from_viewpoint(viewpoint)





if __name__ == '__main__':
    filename = '007.obj'
    so = CameraProject(filename, N=100)
    so.run()

