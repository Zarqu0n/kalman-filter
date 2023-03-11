import numpy as np

class EKF:
    """
    Giriş parametreleri
    ----------
    dim_x : int
        Durum değişkeni sayımız.
    
    Özellikler
    ----------
    x : np.ndarray
        Durum değişkenleri matrisi
    P : np.ndarray
        Durum değişkenleri kovaryans matrisi
    F : np.ndarray
        Durum değişkenleri kat sayı matrisi
    Q : np.ndarray
        İşlem gürültüsü
    H : np.ndarray
        Çıkış katsayı matrisi
    R : np.ndarray
        Ölçüm gürültüsü
    
    """
    def __init__(self,dim_x) -> None:

        # Denklem özellikleri
        self.x = np.zeros((dim_x, 1))       #durum değişkenleri matrisi(hız,pozisyon,...)
        self.P = np.eye(dim_x)              #durum değişkenleri kovaryans matrisi
        self.F = np.eye(dim_x)              #durum değişkenleri kat sayı matrisi
        self.Q = np.zeros((dim_x, dim_x))   #işlem gürültüsü      
        self.H : np.ndarray                 #çıkış katsayı matrisi
        self.R = np.zeros((dim_x, dim_x))   #ölçüm gürültüsü
        self.dim_x = dim_x

        # İşlem özellikleri
        self.y = np.zeros((dim_x, 1))
        self.K = np.zeros((self.x.shape))   #Kalman kazancı

        
    def predict(self) -> None:
        """
        Durum tahmini yapar
        """

        self.x = self.F.dot(self.x)                             #x' = Fx
        self.P = self.F.dot(self.P).dot(self.F.T) + self.Q      #P' = FPF^T + Q

    def update(self, z: np.ndarray,HJacobian,
               Hx,R=None,residual = np.subtract) -> None:
        """
        Ölçüm sonucu ile durum tahmini arasındaki farkı hesaplar

        Giriş parametreleri
        ----------
        z : np.ndarray
            Ölçüm sonucu vektörü

        HJacobian : function
            Çıkış katsayı matrisinin Jacobian matrisini hesaplayan fonksiyon

        Hx : function
            Durum değişkenlerini çıkış değişkenlerine dönüştüren fonksiyon

        R : np.ndarray
            Ölçüm gürültüsü matrisi (varsayılan: self.R)

        residual : function
            Ölçüm sonucu ile tahmin sonucu arasındaki farkı hesaplayan fonksiyon
            (varsayılan: np.subtract)

        """

        if R is None:
            R = self.R
        
        H = HJacobian(self.x)                                   #Çıkış katsayı matrisinin Jacobian matrisi
        S = H.dot(self.P).dot(H.T) + self.R                     #S = HPH^T + R
        K = self.P.dot(H.T).dot(np.linalg.inv(S))               #Kalman kazancı = PH^T(S^-1)
        hx = Hx(self.x)     
        self.y = residual(z, hx)                                #Ölçüm sonucu ile tahmin sonucu arasındaki fark = z - hx
        
        self.x = self.x + K.dot(self.y)                         #Durum değişkenleri = x + K(z - hx)
        self.P = (np.eye(self.dim_x) - K.dot(H)).dot(self.P)    #Durum değişkenleri kovaryans matrisi = (I - KH)P
