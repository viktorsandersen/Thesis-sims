import mujoco
import spatialmath as sm
import roboticstoolbox as rtb
import numpy as np
import mujoco_viewer

# Create robot class for the ur5e robot for the robotics toolbox
class Ur5e:
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path('universal_robots_ur5e/scene.xml')
        self.d = mujoco.MjData(self.m)
        self.joints = [0,0,0,0,0,0,0]
        self.dt = 0.001                 # Time step, should be the same as the mujoco simulation
        self.geom_names       = [mujoco.mj_id2name(self.m,mujoco.mjtObj.mjOBJ_GEOM,x)
                                for x in range(self.m.ngeom)]
        # Universal Robot UR5e kiematics parameters
        tool_matrix = sm.SE3.Trans(0., 0., 0.05)
        robot_base = sm.SE3.Trans(0,0,0)

        self.q0=[0 , -np.pi/2, -np.pi/2, -np.pi/2, np.pi/2,0] # Home position
        self.q_traj0 = [ 2.61089568, -1.54542687,  1.28962333, -1.31499295, -1.57079622,  1.04009936]
        #[6.13010218e-04, -2.82543728e+00,  1.28962314e+00,  3.10661046e+00, 1.57079633e+00,  1.57140934e+00]
        

        self.robot = rtb.DHRobot(
            [ 
                rtb.RevoluteDH(d=0.163, alpha = np.pi/2),
                rtb.RevoluteDH(a=-0.425),
                rtb.RevoluteDH(a=-0.392),
                rtb.RevoluteDH(d=0.127, alpha=np.pi/2),
                rtb.RevoluteDH(d=0.1, alpha=-np.pi/2),
                rtb.RevoluteDH(d=0.1)
            ], name="UR5e",
            base=robot_base,
            tool = tool_matrix,
            )
        
        #used for draw arrow
        self._markers = []

    def add_marker(self, **marker_params):
        self._markers.append(marker_params)

    def invkin(self, via_points, q=None):
        return self.robot.ikine_LM(via_points, q0=q)
    
    def fkine(self, q):
        return self.robot.fkine(q)
    
    def jacob0(self, q):
        return self.robot.jacob0(q)
    
    def ajacob0(self, q, representation='eul'):
        return self.robot.jacob0_analytical(q, representation=representation)
    
    def djacob0(self, q, dq , representation='eul'):
        return self.robot.jacob0_dot(q, dq, representation=representation)
    
    def init_viewer(self,viewer_title='MuJoCo',viewer_width=1200,viewer_height=800,viewer_hide_menus=True):
        """
            Initialize viewer
        """
        self.USE_MUJOCO_VIEWER = True
        self.viewer = mujoco_viewer.MujocoViewer(
                self.m,self.d,mode='window',title=viewer_title,
                width=viewer_width,height=viewer_height,hide_menus=viewer_hide_menus)

    def get_contact_info(self,must_include_prefix=None):
        """
            Get contact information
        """
        p_contacts = []
        f_contacts = []
        p_distance = []
        geom1s = []
        geom2s = []
        for c_idx in range(self.d.ncon):
            contact = self.d.contact[c_idx]
            distance = contact.dist  # Contact distance
            # Contact position and frame orientation
            p_contact = contact.pos # contact position
            R_frame = contact.frame.reshape((3,3))
            # Contact force
            f_contact_local = np.zeros(6,dtype=np.float64)
            mujoco.mj_contactForce(self.m,self.d,0,f_contact_local)
            f_contact = R_frame @ f_contact_local[:3] # in the global coordinate
            # Contacting geoms
            contact_geom1 = self.geom_names[contact.geom1]
            contact_geom2 = self.geom_names[contact.geom2]
            # Append
            if must_include_prefix is not None:
                if (contact_geom1[:len(must_include_prefix)] == must_include_prefix) or (contact_geom2[:len(must_include_prefix)] == must_include_prefix):
                    p_distance.append(distance)
                    p_contacts.append(p_contact)
                    f_contacts.append(f_contact)
                    geom1s.append(contact_geom1)
                    geom2s.append(contact_geom2)
            else:
                p_distance.append(distance)
                p_contacts.append(p_contact)
                f_contacts.append(f_contact)
                geom1s.append(contact_geom1)
                geom2s.append(contact_geom2)
        return p_contacts,f_contacts,geom1s,geom2s,p_distance
    
    def plot_arrow(self,p,uv,r_stem=0.03,len_arrow=0.3,rgba=[1,0,0,1],label=''):
        """
            Plot arrow
        """
        p_a = np.copy(np.array([0,0,1]))
        p_b = np.copy(uv)
        p_a_norm = np.linalg.norm(p_a)
        p_b_norm = np.linalg.norm(p_b)
        if p_a_norm > 1e-9: p_a = p_a/p_a_norm
        if p_b_norm > 1e-9: p_b = p_b/p_b_norm
        v = np.cross(p_a,p_b)
        S = np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
        if np.linalg.norm(v) == 0:
            R = np.eye(3,3)
        else:
            R = np.eye(3,3) + S + S@S*(1-np.dot(p_a,p_b))/(np.linalg.norm(v)*np.linalg.norm(v))

        self.viewer.add_marker(
            pos   = p,
            mat   = R,
            type  = mujoco.mjtGeom.mjGEOM_ARROW,
            size  = [r_stem,r_stem,len_arrow],
            rgba  = rgba,
            label = label
        )