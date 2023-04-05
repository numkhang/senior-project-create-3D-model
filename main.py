from tkinter import *
import tkinter  as tk
from tkinter import messagebox
from tkinter.filedialog import askopenfile
from PIL import Image,ImageTk
from calibration_func import calibration
from stereo import stereo_calculation
from model_func import create_3d_alpha,create_3d_poisson,show_pc
from pointcloud_func import write_ply
from glob import glob
import numpy as np
import os
import cv2 as cv
import time 



# global
K = "-"
distortion = "-"
win_size = "500x800"

np.set_printoptions(suppress=True)

# path of dir of this file
dir = __file__ .split('\\')
del dir[-1]
path = '\\'.join(dir)



def resource_path(relative_path):
    # p = os.path.join(my_path, relative_path)
    return path + relative_path


def upload_calibration():
    intrinsic_drop.config(state=DISABLED)

    f_types = [('Video Files', ['*.mp4'])]
    filename = askopenfile(mode="r",filetypes=f_types)
    if filename :
        global K
        global distortion   
        (K,distortion) = calibration(filename.name)
        label_K_value.configure(text=np.around(K,2))
        label_distortion_value.configure(text=np.around(distortion,2))
        open_save_para_popup(K,distortion)
    else:
        selected_parameter.set("Select Camera parameter")
    intrinsic_drop.config(state=NORMAL)

def upload_stereo():
    try:
        loop_size = int(input_sample.get())
    except:
        messagebox.showerror('Error', 'Error: Please input Integer with value more than zero')
        return
    if K == "-" or distortion == "-" :
        messagebox.showerror('Error', 'Error: Please Calibrate your camera')
    elif loop_size <= 0 :
        messagebox.showerror('Error', 'Error: Please input Integer with value more than zero')
    else:
        pc_drop.config(state=DISABLED)
        f_types = [('Video Files', ['*.mp4'])]
        filename = askopenfile(mode="r",filetypes=f_types)
        if filename :
            start_time = time.time()
            pc = stereo_calculation(filename.name,K,distortion,loop_size)
            fin_time = time.time()
            print(fin_time-start_time)
            if (pc == "error") :
                messagebox.showerror('Error', 'Error: Error in Stereo Calculation Please Retry with other Video')
            elif len(pc) == 0:
                messagebox.showerror('Error', "Error: Empty Point Cloud Please Retry with other Video")
            else:
                open_save_pc_popup(pc)
        else:
            selected_pc.set("Select Point Cloud")
        pc_drop.config(state=NORMAL)
        

def list_dir_in_path(init,path):
    list = glob(f'{path}/*/')
    res = [init]
    for i in list:
        res.append(i.split('\\')[-2])
    return res

def close_popup(popup):
    popup.grab_release()
    popup.destroy()  

def big_chess_image():
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    root.geometry("{}x{}+0+0".format(screen_width-50, screen_height-100))

    label_chess_full.pack(pady=5)
    label_chess.pack_forget()
def small_chess_image():
    root.geometry(win_size)
    label_chess_full.pack_forget()
    label_chess.pack(pady=5)

def open_save_para_popup(K,dis):
    popup = Toplevel(root)
    popup.title("Camera Parameter")
    popup.geometry("300x100")
    entry_name= Entry(popup, width= 100)
    entry_name.insert(INSERT, "Enter Camera Parameter name")
    popup.protocol("WM_DELETE_WINDOW", lambda: close_popup(popup))
    button_save_pop = Button(popup,text="Save",width=10,height=1,command=lambda: save_parameter(popup,entry_name.get(),K,dis))

    entry_name.pack(padx=30,pady=10)
    button_save_pop.pack(pady=10)
    popup.grab_set()

def select_parameter(self):
    global K
    global distortion
    if selected_parameter.get() == "Add New Camera parameter":
        upload_calibration()
    else:
        k_path = '\\intrinsic\\'+selected_parameter.get()+"\\K.npy"
        K = np.load(resource_path(k_path))  
        dist_path  = '\\intrinsic\\'+selected_parameter.get()+"\\distortion.npy" 
        distortion = np.load(resource_path(dist_path))  
        label_K_value.configure(text=np.around(K,2))
        label_distortion_value.configure(text=np.around(distortion,2))
    intrinsic_delete.config(state= NORMAL)

def save_parameter(popup,name,K,dis):
    intrinsic = resource_path('\\intrinsic')
    newpath = os.path.join(intrinsic,name)
    if name == "Enter Camera Parameter name":
        messagebox.showerror('Error', 'Error: Please Enter Parameter name')
    elif name == "Add New Camera parameter" or name == "Select Camera parameter":
        messagebox.showerror('Error', 'Error: Please Choose other name')
    else:
        try:
            os.mkdir(newpath)
            np.save(newpath+"\\K",K)
            np.save(newpath+"\\distortion",dis)
            global intrinsic_dir
            intrinsic_dir = list_dir_in_path("Add New Camera parameter",intrinsic)
            selected_parameter.set(name)
            intrinsic_drop['menu'].delete(0, 'end')
            for choice in intrinsic_dir:
                intrinsic_drop['menu'].add_command(label=choice, command=tk._setit(selected_parameter,choice,select_parameter))
            close_popup(popup) 
        except FileExistsError:
            messagebox.showerror('Error', 'Error: This file already main.exeexist')

def delete_parameter():
    para = selected_parameter.get()
    if para =="Select Camera parameter":
        messagebox.showerror('Error', 'Error: Please Select Parameter to Delete')
        return
    intrinsic = resource_path('\\intrinsic')
    newpath = os.path.join(intrinsic,para)
    for file_name in os.listdir(newpath):
        file = os.path.join(newpath,file_name)
        if os.path.isfile(file):
            os.remove(file)
    os.rmdir(newpath)
    K = "-"
    distortion = "-"
    label_K_value.configure(text="-")
    label_distortion_value.configure(text="-")

    intrinsic_dir = list_dir_in_path("Add New Camera parameter",intrinsic)
    selected_parameter.set("Select Camera parameter")
    intrinsic_drop['menu'].delete(0, 'end')
    for choice in intrinsic_dir:
        intrinsic_drop['menu'].add_command(label=choice, command=tk._setit(selected_parameter,choice,select_parameter))
    intrinsic_delete.config(state= DISABLED)


def open_save_pc_popup(pc):
    popup = Toplevel(root)
    popup.title("PointCloud")
    popup.geometry("300x100")
    entry_name= Entry(popup, width= 100)
    entry_name.insert(INSERT, "Enter PointCloud name")
    popup.protocol("WM_DELETE_WINDOW", lambda: close_popup(popup))
    button_save_pop = Button(popup,text="Save",width=10,height=1,command=lambda: save_pc(popup,entry_name.get(),pc))

    entry_name.pack(padx=30,pady=10)
    button_save_pop.pack(pady=10)
    popup.grab_set()

def select_pc(self):
    if selected_pc.get() == "Create New Point Cloud":
        upload_stereo()
    pc_delete.config(state= NORMAL)

def save_pc(popup,name,pc):
    res_path = resource_path('\\result_file')
    newpath = os.path.join(res_path,name)
    if name == "Enter PointCloud name":
        messagebox.showerror('Error', 'Error: Please Enter Point Cloud name')
    elif name == "Create New Point Cloud":
        messagebox.showerror('Error', 'Error: Please Choose other name')
    else:
        try:
            os.mkdir(newpath)
            write_ply("pointcloud.ply",newpath,pc)
            global res_dir
            res_dir = list_dir_in_path("Create New Point Cloud",res_path)
            selected_pc.set(name)
            pc_drop['menu'].delete(0, 'end')
            for choice in res_dir:
                pc_drop['menu'].add_command(label=choice, command=tk._setit(selected_pc,choice,select_pc))
            close_popup(popup) 
        except FileExistsError:
            messagebox.showerror('Error', 'Error: This file already exist')

def delete_pc():
    pc = selected_pc.get()
    if pc =="Select Point Cloud":
        messagebox.showerror('Error', 'Error: Please Select Point Cloud to Delete')
        return
    res_path = resource_path('\\result_file')
    newpath = os.path.join(res_path,pc)
    for file_name in os.listdir(newpath):
        file = os.path.join(newpath,file_name)
        if os.path.isfile(file):
            os.remove(file)
    os.rmdir(newpath)

    res_dir = list_dir_in_path("Create New Point Cloud",res_path)
    selected_pc.set("Select Point Cloud")
    pc_drop['menu'].delete(0, 'end')
    for choice in res_dir:
        pc_drop['menu'].add_command(label=choice, command=tk._setit(selected_pc,choice,select_pc))
    pc_delete.config(state= DISABLED)
    

def show_point_cloud():
    if selected_pc.get() == "Select Point Cloud":
        messagebox.showerror('Error', 'Error: Please Select Point Cloud')
    else:
        try:
            nb_points = int (input_nb_points.get())
            radius = int (input_radius.get())
            voxel = int(input_down_sample.get())
            res_path = resource_path('\\result_file')
            res_path = os.path.join(res_path,selected_pc.get())
            pc = show_pc(res_path,voxel,nb_points,radius)
            root.geometry(win_size) 
        except:
            messagebox.showerror('Error', 'Error: Invalid Parameter Input')

def create_alpha():
    if selected_pc.get() == "Select Point Cloud":
        messagebox.showerror('Error', 'Error: Please Select Point Cloud')
    else:
        try:
            nb_points = int (input_nb_points.get())
            radius = int (input_radius.get())
            alpha = float (input_alpha.get())
            voxel = int(input_down_sample.get())
            res_path = resource_path('\\result_file')
            res_path = os.path.join(res_path,selected_pc.get())
            mesh = create_3d_alpha(res_path,voxel,nb_points,radius,alpha)
            root.geometry(win_size) 
        except:
            messagebox.showerror('Error', 'Error: Invalid Parameter Input')

    
def create_poisson():
    if selected_pc.get() == "Select Point Cloud":
        messagebox.showerror('Error', 'Error: Please Select Point Cloud')
    else:
        try:
            nb_points = int (input_nb_points.get())
            radius = int (input_radius.get())
            voxel = int(input_down_sample.get())
            depth = int (input_depth.get())
            width = float (input_width.get())
            denst = float (input_denst.get())
            if denst > 1 or denst < 0 :
                messagebox.showerror('Error', 'Error: denstity quantile have a value between [0,1]')
            else:
                res_path = resource_path('\\result_file')
                res_path = os.path.join(res_path,selected_pc.get())
                mesh = create_3d_poisson(res_path,voxel,nb_points,radius,depth,width,denst)
                root.geometry(win_size) 
        except:
            messagebox.showerror('Error', 'Error: Invalid Parameter Input')

    



root =Tk()
root.title("Create 3D Model")   
root.geometry(win_size) 
# root.resizable(True,True)
# root.resizable(False,False)

main_frame = Frame(root)

intrinsic_frame = Frame(main_frame,highlightbackground="gray", highlightthickness=2,width=500,height=300,)
label_intrinsic_frame_header = Label(intrinsic_frame,text="Camera Calibration",font=("Arial", 15))

chess_img_frame = Frame(intrinsic_frame)

chess_pat_path = resource_path("\\resource\\chess_pattern.jpg")
chess_pat = cv.imread(chess_pat_path)
pic_size =( int(chess_pat.shape[1]/14),int(chess_pat.shape[0]/14))
chess = ImageTk.PhotoImage(Image.open(chess_pat_path).resize(pic_size))
label_chess = Label(chess_img_frame,image=chess)
label_chess.bind("<Button-1>",lambda e:big_chess_image())

chess_full = ImageTk.PhotoImage(Image.open(chess_pat_path))
label_chess_full = Label(chess_img_frame,image=chess_full)
label_chess_full.bind("<Button-1>",lambda e:small_chess_image())

drop_frame = Frame(intrinsic_frame)

intrinsic_dir = list_dir_in_path("Add New Camera parameter",resource_path('\\intrinsic'))
selected_parameter = StringVar(value="Select Camera parameter")
intrinsic_drop = OptionMenu( drop_frame , selected_parameter ,*intrinsic_dir,command=select_parameter)
intrinsic_drop.config(width=30,height=2)
intrinsic_delete = Button(drop_frame,text="Delete",width=10,height=2,command=delete_parameter,state= DISABLED)



K_frame = Frame(intrinsic_frame,padx=20,pady=20)
label_K = Label(K_frame,text="K",font=("Arial", 15))

dis_frame = Frame(intrinsic_frame,)
label_distortion = Label(dis_frame,text="distortion",font=("Arial", 15))

label_K_value = Label(K_frame,text=K,font=("Arial", 10))
label_distortion_value = Label(dis_frame,text=distortion,font=("Arial", 10))

stereo_frame = Frame(main_frame,highlightbackground="gray", highlightthickness=2)
label_stereo_frame_header = Label(stereo_frame,text="Create 3D Model",font=("Arial", 15))

sample_frame = Frame(stereo_frame)
label_sample = Label(sample_frame,text="Number of Selected Frame:",font=("Arial", 10))
input_sample = Entry(sample_frame,width=20)
input_sample.insert(0,"200")

pc_drop_frame = Frame(stereo_frame)
res_dir = list_dir_in_path("Create New Point Cloud",resource_path('\\result_file'))
selected_pc = StringVar(value="Select Point Cloud")
pc_drop = OptionMenu( pc_drop_frame , selected_pc ,*res_dir,command=select_pc)
pc_drop.config(width=30,height=2)
pc_delete = Button(pc_drop_frame,text="Delete",width=10,height=2,command=delete_pc,state= DISABLED)


pc_fram = Frame(stereo_frame)
label_outlier = Label(pc_fram,text="Parameter for Down Sample & Outlier Removal",font=("Arial", 10))
pc_para_frame = Frame(pc_fram)

down_sample_frame = Frame(pc_para_frame)
label_down_sample = Label(down_sample_frame,text="voxel size:",font=("Arial", 10))
input_down_sample = Entry(down_sample_frame,width=10)
input_down_sample.insert(0,"1")

nb_points_frame = Frame(pc_para_frame)
label_nb_points = Label(nb_points_frame,text="nearby points:",font=("Arial", 10))
input_nb_points = Entry(nb_points_frame,width=10)
input_nb_points.insert(0,"5")

radius_frame = Frame(pc_para_frame)
label_radius = Label(radius_frame,text="radius:",font=("Arial", 10))
input_radius = Entry(radius_frame,width=10)
input_radius.insert(0,"10")

button_view_pc = Button(pc_fram,text="View Point Cloud",width=15,height=2,command=show_point_cloud) 


mesh_frame = Frame(stereo_frame)
label_choose_3d = Label(mesh_frame,text="Choose method to Create 3D Mesh",font=("Arial", 10))

alphashape_frame = Frame(mesh_frame)
poisson_frame = Frame(mesh_frame)

alpha_frame = Frame(alphashape_frame)
label_alpha= Label(alpha_frame,text="alpha:",font=("Arial", 10))
input_alpha = Entry(alpha_frame,width=10)
input_alpha.insert(0,"10")


depth_frame = Frame(poisson_frame)
label_depth= Label(depth_frame,text="depth:",font=("Arial", 10))
input_depth = Entry(depth_frame,width=10)
input_depth.insert(0,"20")

width_frame = Frame(poisson_frame)
label_width= Label(width_frame,text="width:",font=("Arial", 10))
input_width = Entry(width_frame,width=10)
input_width.insert(0,"5")

denst_frame = Frame(poisson_frame)
label_denst= Label(denst_frame,text="denstity quantile:",font=("Arial", 10))
input_denst = Entry(denst_frame,width=10)
input_denst.insert(0,"0.01")

create_frame = Frame(stereo_frame)

button_alpha = Button(create_frame,text="Create Alpha shape",width=20,height=2,command=create_alpha) 
button_poisson = Button(create_frame,text="Create Poisson Surface",width=20,height=2,command=create_poisson) 



main_frame.pack()
intrinsic_frame.pack(padx=10,pady=10,expand=True)
label_intrinsic_frame_header.pack(pady=5)
chess_img_frame.pack()
label_chess.pack(pady=5)

drop_frame.pack(pady=10)
intrinsic_drop.pack(padx=5,side=LEFT)
intrinsic_delete.pack(padx=5,side=LEFT)

K_frame.pack(pady=5)
dis_frame.pack(pady=5)
label_K.pack(padx=10,side=LEFT)
label_distortion.pack(padx=10,side=LEFT)
label_K_value.pack(padx=10,side=LEFT)
label_distortion_value.pack(padx=10,side=LEFT)

stereo_frame.pack(padx=10,pady=10,expand=True)
label_stereo_frame_header.pack(pady=10,padx=85)

sample_frame.pack(pady=5)
label_sample.pack(padx=5,side=LEFT)
input_sample.pack(padx=5,side=LEFT)

pc_drop_frame.pack(pady=5)
pc_drop.pack(padx=5,side=LEFT)
pc_delete.pack(padx=5,side=LEFT)


pc_fram.pack(padx=10,pady=10)
label_outlier.pack(pady=5)
pc_para_frame.pack(side=LEFT)

down_sample_frame.pack()
label_down_sample.pack(padx=10 ,side=LEFT)
input_down_sample.pack(padx=10 ,side=LEFT)

nb_points_frame.pack()
label_nb_points.pack(padx=10 ,side=LEFT)
input_nb_points.pack(padx=10 ,side=LEFT)

radius_frame.pack()
label_radius.pack(padx=10 ,side=LEFT)
input_radius.pack(padx=10 ,side=LEFT)

button_view_pc.pack(padx=10 ,side=RIGHT)

mesh_frame.pack(padx=10,pady=10)
label_choose_3d.pack(pady=5)
alphashape_frame.pack(side=LEFT)
poisson_frame.pack(side=RIGHT)

alpha_frame.pack()
label_alpha.pack(padx=10 ,side=LEFT)
input_alpha.pack(padx=10 ,side=LEFT)

depth_frame.pack()
label_depth.pack(padx=30 ,side=LEFT)
input_depth.pack(padx=10 ,side=LEFT)

width_frame.pack()
label_width.pack(padx=30 ,side=LEFT)
input_width.pack(padx=10 ,side=LEFT)

denst_frame.pack()
label_denst.pack(padx=5 ,side=LEFT)
input_denst.pack(padx=10 ,side=LEFT)

create_frame.pack(pady=5)
button_alpha.pack(padx=5,side=LEFT)
button_poisson.pack(padx=5,side=RIGHT)

root.mainloop()
