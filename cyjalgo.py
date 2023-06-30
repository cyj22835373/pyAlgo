import numpy as np
from math import *
import pandas as pd
import cv2
import akshare as ak
import datetime
from datetime import datetime
import os
from configparser import ConfigParser

def MyAlgoTest():
    print('hello myalgo!')
def ReadPara(path,InOrEx):
    if InOrEx=='in':
        f=open(path+'inparam.txt','r')
    elif InOrEx=='ex':
        f=open(path+'exparam.txt','r')
    lines= f.readlines()
    lines=lines[0:3]
    a=[]
    for l in lines:
        a.extend(l.split())
    return list(map(float,a))
def If3dPosIsInArea(pt,area):
    minx=min(area[0][0],area[2][0]);maxx=max(area[0][0],area[2][0])
    minz=min(area[0][2],area[2][2]);maxz=max(area[0][2],area[2][2])
    if ((pt[0]>= minx) and (pt[0]<=maxx)) and ((pt[2]>= minz) and (pt[2]<=maxz)):
        return True
    else:
        return False
def getIOU(r1,r2):
    left=max(r1[0],r2[0]);right=min(r1[0]+r1[2],r2[0]+r2[2])
    up=max(r1[1],r2[1]);bottom=min(r1[1]+r1[3],r2[1]+r2[3])
    if (right-left)<0 or (bottom-up)<0:
        return 0.0
    else:
        cross=(right-left)*(bottom-up)
        return cross/(r1[2]*r1[3]+r2[2]*r2[3]-cross)
def ObjlistMultiTrack(mulobjlist,newobjlist,iouthr,multhrH,multhrL):

    for i,mo in enumerate(mulobjlist):
        ious=[]
        for no in newobjlist:
            if no.conbined==True:
                ious.append(0.0)
            else:
                ious.append(mo.GetIOU(no))
        maxv=max(ious)
        #print('out',mulobjlist[0].rect)
        if maxv > iouthr:
            
            maxindex=ious.index(maxv)
            newobjlist[maxindex].conbined=True
            mo.conbined=True
            mo.UpdataInfo(newobjlist[maxindex],0.04)
            mo.conf=min(mo.conf,multhrH)
        else:
            mo.conf=mo.conf-1


    for no in newobjlist:
        if no.conbined==False:
            mulobjlist.append(no)
    for i,mo in enumerate(mulobjlist):
        if mo.conf < multhrL:
            mulobjlist.pop(i)
def normalization(data):
    datamax=np.max(data)
    datamin=np.min(data)
    #print(datamax,datamin)
    _range = datamax - datamin
    #print(datamin,_range)
    return (data - datamin) / _range,datamax,datamin

         

         

class object():
    rect=[0,0,0,0]
    type=0
    pos=[0,0,0]
    area=0
    v=0.0
    prepos=[0,0,0]
    mag=1.0
    firstpos=[0,0,0]
    dirction=0.0
    time=0.0
    conf=0
    conbined=False
    pz=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    vv=[]
    def __init__(self,rect,pos,area,type):
        self.rect=rect
        self.type=type
        self.pos=pos
        self.firstpos=pos
        self.prepos=pos
        self.area=area
        self.pz=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    def GetIOU(self,obj):
        return getIOU(self.rect,obj.rect)
    def MatchAnObj(self,obj,thr):
        if getIOU(self.rect,obj.rect)>thr:
            return True
        else:
            return False
    def UpdataInfo(self,newobj,period):
        #print('in update',self.pos[:],newobj.pos[:])
        #print(self.rect,newobj.rect,self.pos,newobj.pos)
        self.rect=newobj.rect
        self.pos=newobj.pos
 
        self.pz[0:len(self.pz)-1]=self.pz[1:len(self.pz)];self.pz[len(self.pz)-1]=self.pos[2]
       
        self.area=newobj.area
        mov=self.pos-self.prepos
        #print(self.pos,self.prepos)
        #print(mov)
        self.dirction=atan2(mov[0],mov[2])
        if self.conf<10:
            self.v=(self.pos[2]-self.firstpos[2])/(self.time+period)/1000*3.6
        else: 
            self.v=(self.pz[19]-self.pz[10])/(10*period)/1000*3.6
        # self.vv.append(mov[2]/period/1000*3.6)
        # if len(self.vv)>20:
        #     del self.vv[:-20]
        # self.v=sum(self.vv)/len(self.vv)
        #print(self.v)
        #print(self.v,self.vv)
        self.time=self.time+period
        self.prepos=self.pos
        self.conf=self.conf+1
    def drawobj(self,img):
        #print(self.rect[0],self.rect[1],self.rect[0]+self.rect[2],self.rect[1]+self.rect[3])
#        cv2.rectangle(img,(self.rect[0],self.rect[1]),(self.rect[0]+self.rect[2],self.rect[1]+self.rect[3]),(255,255,0),1)
        #cv2.putText(img,str(self.type)+'_'+str(int(self.v))+'_'+str(int(self.dirction))+'_'+str(self.conf)+'_'+str(int(self.pos[0]))+'_'+str(int(self.pos[2])),(self.rect[0],self.rect[1]),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,0),1)
#        cv2.putText(img,str(self.type)+'_'+str(int(self.v)),(self.rect[0],self.rect[1]),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1)
        return img

class CamModel():
    inparas=[]
    exparas=[]
    cx=640
    cy=360
    k=[]
    asp=1.0
    rx=0.0
    ry=0.0
    rz=0.0
    tx=0.0
    ty=-600.0
    tz=0.0
    RMat=np.empty((3,3))
    MaxRadius=0.0
    def __init__(self,InParams,OutParams):
        self.inparas=InParams
        self.exparas=OutParams
        self.cx=InParams[0]
        self.cy=InParams[1]
        self.k=InParams[2:7]
        self.asp=InParams[7]
        self.tx=OutParams[0]
        self.ty=OutParams[1]
        self.tz=OutParams[2]
        self.rx=OutParams[3]
        self.ry=OutParams[4]
        self.rz=OutParams[5]
        rx=radians(self.rx);ry=radians(self.ry);rz=radians(self.rz)
        RM=[i for i in range(9)]
        RM[0]=cos(ry)*cos(rz);RM[1]=-cos(ry)*sin(rz);RM[2]=sin(ry)
        RM[3]=sin(rx)*sin(ry)*cos(rz) + cos(rx)*sin(rz);RM[4]=cos(rx)*cos(rz) - sin(rx)*sin(ry)*sin(rz);RM[5]=-sin(rx)*cos(ry)
        RM[6]=sin(rx)*sin(rz) - cos(rx)*sin(ry)*cos(rz);RM[7]=sin(rx)*cos(rz) + cos(rx)*sin(ry)*sin(rz);RM[8]=cos(rx)*cos(ry)
        self.RMat=np.array(RM).reshape(3,3).T.copy()
        #k1=self.k[0];k2=self.k[1];k3=self.k[2];k4=self.k[3];k5=self.k[4]
        k=np.array(self.k)
        square=np.array([0.0,0.0,0.0,0.0,0.0])
        num = int(pi/2/0.05)+1
        _max=0.0
        for i in range(num):
            if i==0:
                x=0.0;x_1=0.0
            else:
                x = x_1 + 0.05;x_1 = x
            square[0]=x;square[1]=square[0]*x*x;square[2]=square[1]*x*x;square[3]=square[2]*x*x;square[4]=square[3]*x*x
            y=np.dot(k,square)
            if y>_max:
                _max=y
        self.MaxRadius=_max
    def GetTheta(self,r):
        counter = 0;tPre = 1.0;diff = 999.0;tCur = 0.0;precision = 0.01
        s=np.array([0.0,0.0,0.0,0.0,0.0]);k=np.array(self.k);_max=self.MaxRadius
        if r>_max:
            return 0,pi/2
        else:
            while diff>precision:
                s[0]=tPre
                s[1]=s[0]*tPre*tPre
                s[2]=s[1]*tPre*tPre
                s[3]=s[2]*tPre*tPre
                s[4]=s[3]*tPre*tPre
                a=np.dot(s,k)-r;b = k[0] + 3*k[1]*s[0]*tPre + 5*k[2]*s[1]*tPre + 7*k[3]*s[2]*tPre + 9*k[4]*s[3]*tPre
                tCur = tPre-a/b
                diff=fabs(tCur-tPre)
                tPre = tCur
                counter=counter+1
                if counter>100:
                    break
        return 1,tCur
    def Cam2Dto3D(self,p2d,toEarth):
        u,v=((p2d[0]-self.cx)*self.asp,p2d[1]-self.cy)
        R=sqrt(u*u+v*v)
        l,theta=self.GetTheta(R)
        if(l==0):
            return 0,np.array([0,0,0])
        R1=tan(theta);phi=atan2(v,u)
        CorrectedX = R1*cos(phi);CorrectedY = R1 * sin(phi)
        p3d=np.dot(np.array([CorrectedX,CorrectedY,1]),self.RMat)*1000
        if toEarth==True:
            if p3d[1] < 0:
                return 0,p3d
            scale = -self.ty/p3d[1]
            p3d[1]=(int)(-self.ty)
            p3d[0]=(int)(p3d[0]*scale)
            p3d[2]=(int)(p3d[2]*scale)
        return 1,p3d
    def W3Dto2D(self,p3d):
        t=np.dot(self.RMat,np.array(p3d).reshape(3,1))
        #print(t)
        if t[2]<=0.0:
            t[2]=0.0
        t1= atan(sqrt(t[0]*t[0]+t[1]*t[1])/t[2])
        t2= np.dot(self.k,np.array([t1,pow(t1,3),pow(t1,5),pow(t1,7),pow(t1,9)]))
        return np.array([t2*cos(atan2(t[1],t[0]))/self.asp+self.cx,t2*sin(atan2(t[1],t[0]))+self.cy])

####################################################################Annex Fuctions###############################################################
def GetToday():
    import time
    today = time.strftime("%Y-%m-%d")
    return today
def Date2Str(date):
    if isinstance(date,str)==True: #isinstance可以判断数据类型是否已与第二个参数相同
        return date
    d=date.strftime("%Y-%m-%d")
    return d
def Str2Date(date):
    d = datetime.strptime(date,"%Y-%m-%d")
    return d
def GetAvgArray(A,avgNum,type='valid'):
    core=np.ones(avgNum)
    if type=='valid':
        output=np.zeros(len(A))
        #print(A,core,type,avgNum)
        temp=np.convolve(A,core,mode=type)/avgNum
        output[avgNum-1:]=temp[:]
    elif type=='same':
        output=np.convolve(A,core,mode=type)/avgNum
    return output
def GetFileNameList(Path,CutSuffix='no'):
    import os
    for k in enumerate(os.walk(Path)):
        filesname=k[1][2]
    if CutSuffix=='no':
        return filesname
    else:

        for i,f in enumerate(filesname):
            f=f[:len(f)-len(CutSuffix)]
            filesname[i]=f
        return filesname

def GetNAvyArray(A,avgNumList,type='valid'):
    avgListLength=len(avgNumList)
    output=np.zeros((len(A),avgListLength))
    for i,avgNum in enumerate(avgNumList):
        #print(avgNum,A,type)
        temp=GetAvgArray(A,avgNum,type=type)
        output[:,i]=temp[:]
    return output
def AddNAvg2ColsAndCutInvalid(InPut,FilterCol,avgNumList):
    maxAvgNum=np.max(avgNumList)
    AddedData=GetNAvyArray(InPut[:,FilterCol],avgNumList)
    temp=np.append(InPut,AddedData,axis=1)
    temp=temp[maxAvgNum-1:,:]
    return temp
def ReadAndReFormCSV(path):
    df=pd.read_csv(path)
    cname=df.columns[0]
    df.index=df[cname]
    df=df.drop(cname,axis=1)
    return df
def GetStocksDataWithNamelist_Akshare_SameLength(nlist,startdate,enddate='2022-12-31',datasource='net'):
    length=0;dflist=[];tempDF=0
    #print(len(nlist))
    EndOfDate=Date2Str(enddate)
    namelist=[]
    for i,n in enumerate(nlist):
        if datasource=='net':
            tempDF=ak.stock_us_daily(symbol=n, adjust="qfq")
        else :
            tempDF=ReadAndReFormCSV('data/stocktestdata/'+n+'.csv')
      
        tempDF=tempDF[(tempDF.index>startdate) & (tempDF.index < EndOfDate)]
        if i==0:
            length=len(tempDF)
        else:
            if len(tempDF)!=length:
                continue
        dflist.append(tempDF)
        namelist.append(n)
    return dflist,namelist

def UpdateUsStocksData(nlist,basepath):
    for i,n in enumerate(nlist):
        tempDF=ak.stock_us_daily(symbol=n, adjust="qfq")
        tempDF.to_csv(basepath+n+'.csv')
        print(n,i)
        
import requests   
import json
import time

def write_to_ini(key, string, ini_file):
    # 创建 ConfigParser 对象
    config = ConfigParser()

    # 读取 .ini 文件
    config.read(ini_file)

    # 检查 'TempParas' section 是否存在，如果不存在则添加
    if not config.has_section('TempParas'):
        config.add_section('TempParas')

    # 将字符串写入 'TempParas' section 下的键
    config.set('TempParas', key, string)

    # 写入 .ini 文件
    with open(ini_file, 'w') as f:
        config.write(f)
class IPer():
    ipurl = 'http://ip-api.com/json/'
    defaultUtlType = 'ipApi'
    localIP = '172.22.0.1'
    clash_secret = '1223e104-3f56-48fb-8e29-d62fb5ef4f05'
    clash_Port = 'http://127.0.0.1:12082/'

    
    @staticmethod
    def send_request(url, method, data=None):
        if method == 'GET':
            response = requests.get(url)
        elif method == 'POST':
            response = requests.post(url, data=data)
        else:
            print("Invalid method. Please choose either 'GET' or 'POST'.")
            return

        return response
    @staticmethod
    def SetHttpProxy(httpProxy):
        os.environ['http_proxy'] = httpProxy
        os.environ['https_proxy'] = httpProxy 
    @staticmethod
    def ReSetHttpProxy():
        os.environ.pop('http_proxy', None)
        os.environ.pop('https_proxy', None)
    @staticmethod      
    def GetIpOnInternet():
        start_time = time.time()
        try:
            response = requests.get('https://httpbin.org/ip')
        except requests.exceptions.SSLError:
            ans = "No Connection"
            #print("No Connection")
            end_time = time.time()
            return ans,format(end_time - start_time)
        end_time = time.time()
        try:
            ans = response.json()['origin']
        except json.JSONDecodeError:
            ans = "Error"
            print("JSONDecodeError")
        return ans,format(end_time - start_time)
    def SetGetIpUrl(self, url):
        self.ipurl = url
    def SetClashParams(self,secret,ipPort):
        self.clash_secret = self.clash_secret
        self.clash_Port = ipPort
    def GetClashProxies(self):
        api_url = self.clash_Port + 'proxies'
        headers = {
            'Authorization': f'Bearer {self.clash_secret}',
            'Content-Type': 'application/json',
        }
        # 发送 GET 请求
        response = requests.get(api_url, headers=headers)
        return response.json()
    def GetProxiesNameFormSelecter(self,selectoerName,area = None):
        a=self.GetClashProxies()
        plist = a['proxies'][selectoerName]['all']
        delaylist = []
        totallist = []
        filterlist = []
        for i in plist:
            b=a['proxies'][i]
            delaylist.append(b['history'][1]['delay'])
        for i,p in enumerate(plist):
            totallist.append((plist[i],delaylist[i]))
        
        if area == None:
            return totallist
        else:
            for i in totallist:
                if area in i[0]:
                    filterlist.append(i)
            return filterlist
    def GetClashSelecters(self):
        a=self.GetClashProxies()
        selecterList = []
        for p in a['proxies'].keys():
            b=a['proxies'][p]
            if(b['type']=='Selector'):
                selecterList.append(b['name'])
        return selecterList
    def SetClashProxy(self,GoupName,proxyName,clash_secret,clash_Port):
        api_url = clash_Port + 'proxies/'+GoupName
        headers = {
            'Authorization': f'Bearer {clash_secret}',
            'Content-Type': 'application/json',
        }
        data = {
            'name':proxyName,
        }
        response = requests.put(api_url, headers=headers, json=data)
        if response.status_code == 204:
            return 1
        else:
            return -1
    def GetIPinfo(self, ip):
        url = self.ipurl + ip
        method = 'GET'  # 或者 'POST'
        data = {'key': 'value'}  # 如果是POST请求，你可以发送一些数据
        response = self.send_request(url, method, data)
        if self.defaultUtlType =='ipApi':
        
            try:
                response = json.loads(response.text)
            except json.JSONDecodeError:
                return ["error"]
            
            return response
    def GetAnIPFormIp2w():
        return 1
    #申请一个IP
    def GetTodayListFormIp2w(self,localIp):
        url="http://"+localIp+":200/ip/get_todaylist"
        data = {'key': 'value'}  # 如果是POST请求，你可以发送一些数据
        #print(url)
        response = self.send_request(url, "GET", data)
        try:
            response = json.loads(response.text)
        except json.JSONDecodeError:
            return ["error"]
        original_list = response['data']
        response =[d for d in original_list if d['is_online'] != False]
        return response
    def ReBingIp2W(self,localip,ip,port):
        url = "http://"+localip+":200/ip/set_todaylist_session?ip="+ip+"&port="+str(port)
        #print(url)
        method = 'GET'  # 或者 'POST'
        data = {'key': 'value'}  # 如果是POST请求，你可以发送一些数据
        response = self.send_request(url, method, data)
        #response = json.loads(response.text)
        return response
    def RequestIpIp2w_city(self,localip,city='New York City',prot = 45000):
        
        country=""
        state=""
        if city == 'New York City':
            country='US'
            state='New York'
        elif city == 'Hong Kong':
            country='HK'
            state='Central and Western'
        elif city == 'Tokyo':
            country='JP'
            state='Tokyo'
        elif city == 'Toronto':
            
            country='CA'
            state='Ontario'
        elif city == 'Berlin':
            country='DE'
            state='Berlin'
        return self.RequestIpIp2w(localip,country=country,state=state,city=city)
    def RequestIpIp2w(self,localip,country='US',state='New York',city='New York City',isp="random",port=45000):
        url = "http://"+localip+":200/ip/numerous_bind?num=1&country="+country+"&state="+state+"&"+"city="+city+"&"+"isp="+isp+"&zip=random&t=txt&port="+str(port)
        print(url)
        method = 'GET'  # 或者 'POST'
        data = {'key': 'value'}  # 如果是POST请求，你可以发送一些数据
        response = self.send_request(url, method, data)
        return response.text
    def GetCurrentIpIfAvailable(self,ip,proxy = 'http://127.0.0.1:8118',port = 45000):
         #测试当前所有IP的延迟
        self.ReSetHttpProxy()
        iplist = self.GetTodayListFormIp2w(ip) #获得IPlist
        Currentid=0
        if iplist[0] == 'error':
            return ["error"]
        aIpList = []
        
        for i,ipinfo in enumerate(iplist):
            
            if ipinfo['bind_port']==45000:
                print(ipinfo)
                Currentid = i
        print(Currentid)
        ipinfo = iplist[Currentid]
        print(ipinfo)
        max_retries = 3

        for i in range(max_retries):
            a,lay=self.GetIpOnInternet()
            if a != 'No Connection':
                if a == "JSONDecodeError":
                    a, lay = self.GetIpOnInternet()
                    print(a, lay)
                    if a != "JSONDecodeError":
                        self.ReSetHttpProxy()
                        return ipinfo['ip'], lay
                    else:
                        if i < max_retries - 1:  # if it's not the last retry
                            continue  # try again
                        else:  # if it's the last retry
                            self.ReSetHttpProxy()
                            return -1,-1
                else:
                    self.ReSetHttpProxy()
                    return ipinfo['ip'], lay
            else:
                self.ReSetHttpProxy()
                print("No connection")
                return -1,-1

        '''self.SetHttpProxy(proxy)
        a,lay=self.GetIpOnInternet()
        if a != 'No Connection':
            if a == "JSONDecodeError":
                a,lay=self.GetIpOnInternet()
                if a !="JSONDecodeError":
                    self.ReSetHttpProxy()
                    return ipinfo['ip'],lay
                else:
                    self.ReSetHttpProxy()
                    return -1
                     
            else:
               self.ReSetHttpProxy()
               return ipinfo['ip'],lay
        else:
            self.ReSetHttpProxy()
            return -1'''
        self.ReSetHttpProxy()
    def GetAvailableIPs(self,ip,proxy = 'http://127.0.0.1:8118',port = 45000):
        #测试当前所有IP的延迟
        self.ReSetHttpProxy()
        iplist = self.GetTodayListFormIp2w(ip) #获得IPlist
        if iplist[0] == 'error':
            return ["error"]
        aIpList = []
        
        for i,ipinfo in enumerate(iplist):
            if ipinfo['bind_port']==45000:
                Currentid = i
        a=iplist[0]
        iplist[0] = iplist[i]
        iplist[i] = a
        for i,ipinfo in enumerate(iplist):
            print(ipinfo)
            self.ReSetHttpProxy()
            if i>0:
                a=self.ReBingIp2W(ip,ipinfo['ip'],port)
            self.SetHttpProxy(proxy)
            a,lay=self.GetIpOnInternet()
            if a != 'No Connection':
                if a == "JSONDecodeError":
                    a,lay=self.GetIpOnInternet()
                    if a !="JSONDecodeError":
                        aIpList.append((ipinfo['ip'],lay))
                        print(a,lay)
                    else:
                       
                        aIpList.append("JSONDecodeError")
                         
                else:
                    aIpList.append((ipinfo['ip'],lay))
                    print(a,lay)
            else:
                aIpList.append("No Connection")
        self.ReSetHttpProxy()
        return aIpList



        
