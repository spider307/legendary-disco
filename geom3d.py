import numpy as np

EPS = 1e-8

def sign(x):
    if x > EPS:
        return 1
    elif x < -EPS:
        return -1
    else:
        return 0

class Point3():

    def __init__(self,x=0.0,y=0.0,z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def to_list(self):
        return [self.x,self.y,self.z]
    
    def clean(self):
        # 把绝对值小于EPS的数置为0.0
        clean_x = 0.0 if abs(self.x) < EPS else self.x
        clean_y = 0.0 if abs(self.y) < EPS else self.y
        clean_z = 0.0 if abs(self.z) < EPS else self.z
        return Point3(clean_x, clean_y, clean_z)
    
    def __eq__(self, other):
        return (sign(self.x - other.x)==0 and 
                sign(self.y - other.y)==0 and 
                sign(self.z - other.z)==0)

    def __hash__(self):
        return hash((round(self.x, 8), 
                     round(self.y, 8), 
                     round(self.z,8)))
    
    def __sub__(self, other):
        return Point3(self.x-other.x,
                      self.y-other.y,
                      self.z-other.z)
    
    def __add__(self, other):
        return Point3(self.x+other.x,
                      self.y+other.y,
                      self.z+other.z)
    
    def mul(self, k: float):
        """
        三维向量数乘（标量乘法）
        """
        return Point3(self.x * k, self.y * k, self.z * k)

    def __mul__(self, k: float):
        """ 重载 * 运算符 """
        return self.mul(k)
    
    def dot(self,other):
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self,other):
        x =   self.y * other.z - self.z * other.y
        y = -(self.x * other.z - self.z * other.x)
        z =   self.x * other.y - self.y * other.x
        return Point3(x,y,z)
    
    def norm(self):
        return np.sqrt(self.dot(self))
    
    def norm2(self):
        #快速比较大小
        return self.dot(self)
    
    def unit(self):
        #向量单位化
        n = self.norm()
        if sign(n) == 0:
            return Point3(0,0,0)
        return Point3(self.x / n, self.y / n, self.z / n)
    
    def dist_point_segment(self,A,B):
        AB = B - A
        AP = self - A
        #模长平方
        AB_norm_2 = AB.dot(AB)

        if sign(AB_norm_2) == 0:
            return AP.norm()
        
        t = AP.dot(AB) / AB_norm_2

        if t < 0.0:
            #投影角成钝角/直角
            return AP.norm()
        elif t > 1:
            #投影角成锐角
            return (B-self).norm()
        else:
            projection = A + AB*t
            return (projection - self).norm()
    
    def intersect_yoz(self, other):
        """
        求:self 与 other 组成的直线 与 YOZ 平面(x=0)的交点
        返回:Point3 交点
        如果直线平行于 YOZ 平面，返回 None
        """
        x1, y1, z1 = self.x, self.y, self.z
        x2, y2, z2 = other.x, other.y, other.z
        dx = x2 - x1
        # 直线平行于 YOZ，无交点
        if sign(dx) == 0:
            return None
        # 求参数 t，使得 x = 0
        t = -x1 / dx
        # 计算交点坐标
        y = y1 + t * (y2 - y1)
        z = z1 + t * (z2 - z1)
        return Point3(0.0, y, z)
    
# 独立函数：获取 YOZ 平面点集的凸包（返回 Point3 列表）
def convex_hull_yoz(points):
    if len(points) < 3:
        return []
    
    # 转成 (y,z) 坐标
    yz = [(p.y, p.z) for p in points]
    
    # 叉积判断转向
    def cross(o, a, b):
        return (a[0] - o[0])*(b[1] - o[1]) - (a[1] - o[1])*(b[0] - o[0])
    
    # 去重 + 排序
    yz = sorted(list(set(yz)))
    
    # 下凸包
    lower = []
    for p in yz:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= EPS:
            lower.pop()
        lower.append(p)
    
    # 上凸包
    upper = []
    for p in reversed(yz):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= EPS:
            upper.pop()
        upper.append(p)
    
    # 完整凸包点
    hull_yz = lower[:-1] + upper[:-1]
    if len(hull_yz) < 3:
        return []
    
    # 转回 Point3 格式
    hull_points = [Point3(0.0, y, z).clean() for y, z in hull_yz]
    return hull_points

# 新增：YOZ平面凸包面积计算
def convex_hull_area_yoz(hull):
    n = len(hull)
    if n < 3:
        return 0.0
    area = 0.0
    for i in range(n):
        y1, z1 = hull[i].y,hull[i].z
        y2, z2 = hull[(i+1)%n].y, hull[(i+1)%n].z
        area += (y1 * z2) - (y2 * z1)
    return abs(area) / 2.0   
        
# ==============================
# 超快判断：点 P 是否在 凸多边形 内部（YOZ平面）
# ==============================
def is_point_in_convex_polygon(p: Point3, convex_poly: list[Point3]):
    n = len(convex_poly)
    y, z = p.y, p.z

    for i in range(n):
        a = convex_poly[i]
        b = convex_poly[(i+1) % n]
        
        # 叉积：判断点在边的左侧（内部）还是右侧（外部）
        cross_val = (b.y - a.y) * (z - a.z) - (b.z - a.z) * (y - a.y)
        
        # 点在凸多边形外部 → 直接返回 False
        if cross_val < -EPS:
            return False
    
    return True

# ==============================
# 【最快】判断：凸包A 是否 完全包裹 凸包B（均在 YOZ 平面）
# ==============================
def is_convexA_encapsulate_convexB(convexA: list[Point3], convexB: list[Point3]):
    # 凸包必须至少3个点
    if len(convexA) < 3 or len(convexB) < 3:
        return False

    # --------------------------
    # 快速判断：B 的所有顶点 都在 A 内部
    # --------------------------
    for b_point in convexB:
        if not is_point_in_convex_polygon(b_point, convexA):
            return False

    return True  # 全部在内部 → 完全包裹
