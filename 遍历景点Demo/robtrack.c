int gd_f[7];  //前面7个灰度传感器
int gd_b[7];  //后面7个灰度传感器
int flag1;
int flag2;
int flag3;
int flag4;
int flag5;
int com;
int com1;
int com2;
int F_sign[7]={0,0,0,0,0,0,0};
int B_sign[7]={0,0,0,0,0,0,0}; 
int r_s,led;
int gd_l,gd_r;                                  //左右
int gd_threshold_f[7]={429,391,398,386,401,427,422};  
int gd_threshold_b[7]={442,426,369,334,288,267,315};
int gd_threshold_l=154;                         
int gd_threshold_r=100; 
int weight[7]={-12,-8,-4,0,4,8,12};
int r_df,r_db,r_fh,r_af,r_ab;                        //红外壁障    r_af与r_df相关阈值410    r_ab与r_db相关阈值410
int t_fl,t_fr,t_bl,t_br;                        //触碰开关
int t_l,t_r;                                       //左右手触碰开关
int r_xf,r_xb;                                  //斜向下红外测距
int F_state=0;
int F_state_Last=0; 
int F_flag=0;                             
int B_state=0;
int B_state_Last=0;
int B_flag=0;
int turn_flag=0; 
int fast_speed=270;
int slow_speed=220;
int pcs;
int flag[9]={0,0,0,0,0,0,0,0,0};
int xx,yy;
int i=0,n=0;
float T=0.7;
float pt=0.45;
float t=0,tt=0;
int lz_flag1=0,lz_flag2=0;
int r_threshold_f=500,r_threshold_b=500;
int F_num=0,B_num=0;
int qiaoban_speed=200;
int door[6]={0,0,1,1,0,0};
int dirGoal=0;
int dirBuff=0;
//================================================================================//
void go2(int x,int y)
{
  if(x!=xx) motor(1,x); xx=x;
  if(y!=yy) motor(2,y); yy=y;
}

//=============================================================================//

void samp_adc(void)     
{          //模拟口
    int temp_gd_f[7]={0,0,0,0,0,0,0};
    int temp_gd_b[7]={0,0,0,0,0,0,0};
    int temp_gd_l=0,temp_gd_r=0;
    int temp_r_af=0,temp_r_bf=0;
    int i=0;
    for(i=0;i<3;i++)
    {
        temp_gd_f[0]=temp_gd_f[0]+geteadc(1);
        temp_gd_f[1]=temp_gd_f[1]+geteadc(2);
        temp_gd_f[2]=temp_gd_f[2]+geteadc(3);
        temp_gd_f[3]=temp_gd_f[3]+geteadc(4);
        temp_gd_f[4]=temp_gd_f[4]+geteadc(5);
        temp_gd_f[5]=temp_gd_f[5]+geteadc(6);
        temp_gd_f[6]=temp_gd_f[6]+geteadc(7);
        
        temp_gd_b[0]=temp_gd_b[0]+geteadc(15);
        temp_gd_b[1]=temp_gd_b[1]+geteadc(14);
        temp_gd_b[2]=temp_gd_b[2]+geteadc(13);
        temp_gd_b[3]=temp_gd_b[3]+geteadc(12);
        temp_gd_b[4]=temp_gd_b[4]+geteadc(11);
        temp_gd_b[5]=temp_gd_b[5]+geteadc(10);
        temp_gd_b[6]=temp_gd_b[6]+geteadc(9);
        
        temp_gd_l=temp_gd_l+getadc(10);
        temp_gd_r=temp_gd_r+getadc(11);
        
        temp_r_af=temp_r_af+getadc(1);
        temp_r_bf=temp_r_bf+getadc(2);
    }
    
    gd_f[0]=temp_gd_f[0]/3;       
    gd_f[1]=temp_gd_f[1]/3;           
    gd_f[2]=temp_gd_f[2]/3;        
    gd_f[3]=temp_gd_f[3]/3;        
    gd_f[4]=temp_gd_f[4]/3;       
    gd_f[5]=temp_gd_f[5]/3;       
    gd_f[6]=temp_gd_f[6]/3;        
    gd_b[0]=temp_gd_b[0]/3;        
    gd_b[1]=temp_gd_b[1]/3;       
    gd_b[2]=temp_gd_b[2]/3;        
    gd_b[3]=temp_gd_b[3]/3;        
    gd_b[4]=temp_gd_b[4]/3;        
    gd_b[5]=temp_gd_b[5]/3;        
    gd_b[6]=temp_gd_b[6]/3;       
    gd_l=temp_gd_l/3;        
    gd_r=temp_gd_r/3;
    r_af=temp_r_af/3;
    r_ab=temp_r_bf/3;
    
}

void samp_adc_1(void)     
{          //模拟口
    int temp_gd_f[7]={0,0,0,0,0,0,0};
    int temp_gd_b[7]={0,0,0,0,0,0,0};
    int temp_gd_l=0,temp_gd_r=0;
    int temp_r_af=0,temp_r_bf=0;
    int i=0;
    for(i=0;i<3;i++)
    {
        temp_gd_f[0]=temp_gd_f[0]+1024-geteadc(1);
        temp_gd_f[1]=temp_gd_f[1]+1024-geteadc(2);
        temp_gd_f[2]=temp_gd_f[2]+1024-geteadc(3);
        temp_gd_f[3]=temp_gd_f[3]+1024-geteadc(4);
        temp_gd_f[4]=temp_gd_f[4]+1024-geteadc(5);
        temp_gd_f[5]=temp_gd_f[5]+1024-geteadc(6);
        temp_gd_f[6]=temp_gd_f[6]+1024-geteadc(7);
        
        temp_gd_b[0]=temp_gd_b[0]+1024-geteadc(15);
        temp_gd_b[1]=temp_gd_b[1]+1024-geteadc(14);
        temp_gd_b[2]=temp_gd_b[2]+1024-geteadc(13);
        temp_gd_b[3]=temp_gd_b[3]+1024-geteadc(12);
        temp_gd_b[4]=temp_gd_b[4]+1024-geteadc(11);
        temp_gd_b[5]=temp_gd_b[5]+1024-geteadc(10);
        temp_gd_b[6]=temp_gd_b[6]+1024-geteadc(9);
        
        temp_gd_l=temp_gd_l+getadc(10);
        temp_gd_r=temp_gd_r+getadc(11);
        
        temp_r_af=temp_r_af+getadc(1);
        temp_r_bf=temp_r_bf+getadc(2);
    }
    
    gd_f[0]=temp_gd_f[0]/3;       
    gd_f[1]=temp_gd_f[1]/3;           
    gd_f[2]=temp_gd_f[2]/3;        
    gd_f[3]=temp_gd_f[3]/3;        
    gd_f[4]=temp_gd_f[4]/3;       
    gd_f[5]=temp_gd_f[5]/3;       
    gd_f[6]=temp_gd_f[6]/3;        
    gd_b[0]=temp_gd_b[0]/3;        
    gd_b[1]=temp_gd_b[1]/3;       
    gd_b[2]=temp_gd_b[2]/3;        
    gd_b[3]=temp_gd_b[3]/3;        
    gd_b[4]=temp_gd_b[4]/3;        
    gd_b[5]=temp_gd_b[5]/3;        
    gd_b[6]=temp_gd_b[6]/3;       
    gd_l=temp_gd_l/3;        
    gd_r=temp_gd_r/3;
    r_af=temp_r_af/3;
    r_ab=temp_r_bf/3;
    
}

void samp_port(void)      
{          //数字口
    t_fl=getport(4);            //检测到为1
    t_fr=getport(3);
    t_bl=getport(1);
    t_br=!getport(2);                //一直为1，为检测到状态，不要轻易用
    r_df=!getport(8);           //加非号，则检测到障碍物返回1  水平
    r_db=!getport(5);
    r_xf=!getport(7);             //向下
    r_xb=!getport(6);
    r_s=getport(9);
    led=getport(10);
    r_fh=!getport(12);
    com=compass();
    
}
void samp_port2(void)      
{          //数字口
    t_fl=getport(4);            //检测到为1
    t_fr=getport(3);
    t_bl=getport(1);
    t_br=!getport(2);                //一直为1，为检测到状态，不要轻易用
    r_df=!getport(8);           //加非号，则检测到障碍物返回1  水平
    r_db=!getport(5);
    r_xf=!getport(7);             //向下
    r_xb=!getport(6);
    r_s=getport(9);
    led=0;
    r_fh=!getport(12);
    com=compass();
    
}
//============================================================================//
void samp()
{
    samp_adc();
    samp_port();
  //  if(r_t==1) {motor(1,0);motor(2,0);start11();chengxu11();}
com=compass();
}

void samp_1()
{
    samp_adc_1();
    samp_port();
  //  if(r_t==1) {motor(1,0);motor(2,0);start11();chengxu11();}
com=compass();
}
//============================================================================//
void test(void)
{  
    f_state_check();
    b_state_check();
    cls();
    locate(3,1);
    printf("%4d%4d",F_state,F_state_Last);
    locate(5,1);
    printf("%4d%4d",B_state,B_state_Last);   
}

//============================================================================//
int f_state_check(void)
{
    int f[7]={0,0,0,0,0,0,0};
    int temp_buf[7]={0,0,0,0,0,0,0};
    int nf=0;
    int i=0,j=0;
    int total_weight=0;
    int sum=0;
    samp_adc();
    
    for(i=0;i<7;i++)
    {
        if(gd_f[i]<gd_threshold_f[i])  
        {   
            f[i]=1;
            nf++;
        }
        if(f[i]==1)
        {
            temp_buf[j]=i;
            j++;
        }
    }
    F_num=nf;
    j=0;
    
    for(i=0;i<7;i++)
    {
        F_sign[i]=f[i];
    }
    
    if(nf==0) 
    {
        F_flag=1;
        if(F_state_Last<11&&F_state_Last>-11)
        {
            if(F_state_Last<0)      F_state=F_state_Last;
            else if(F_state_Last>0) F_state=F_state_Last;
            else    F_state=F_state_Last;
        }
        else
        {
            if(F_state_Last<0)  F_state=-13;
            else if(F_state_Last>0) F_state=13;
            else    F_state=F_state_Last;
        }
    }
    else if(nf==1)
    {   
        for(i=0;i<7;i++)
            total_weight=total_weight+f[i]*weight[i];
        F_state=total_weight/nf;
        F_flag=0;
    }
    else if(nf==2)
    {
        if((temp_buf[1]-temp_buf[0]==1)||(temp_buf[1]-temp_buf[0]==-1))
        {
            F_flag=0;
            for(i=0;i<7;i++)
                total_weight=total_weight+f[i]*weight[i];
            F_state=total_weight/nf;
        }
        else if(f[3]==1)
        {
            F_flag=0;
            F_state=0;
        }
        else if(abs(temp_buf[1]-3)<=abs(temp_buf[0]-3))
        {
            F_flag=1;
            F_state=weight[temp_buf[1]];
        }
        else if(abs(temp_buf[1]-3)> abs(temp_buf[0]-3))
        {
            F_flag=1;
            F_state=weight[temp_buf[0]];    
        }
        else
        {
            F_flag=1;
            for(i=0;i<7;i++)
                total_weight=total_weight+f[i]*weight[i];
            F_state=total_weight/nf;
        }
    }
    else if(nf==3)
    {
        if(temp_buf[1]==3)
        {
           F_flag=0; 
           F_state=0;
        }
        else if(f[3]==1)
        {
           F_flag=0; 
           F_state=0;
        }
        else if((temp_buf[1]-temp_buf[0]==1)&&(temp_buf[2]-temp_buf[1]==1))
        {
            F_flag=0;
            for(i=0;i<7;i++)
                total_weight=total_weight+f[i]*weight[i];
            F_state=total_weight/nf;
        }
        else
        {
            F_flag=1;
            for(i=0;i<7;i++)
                total_weight=total_weight+f[i]*weight[i];
            F_state=total_weight/nf;
        }
    }
    else if(nf==4)
    {
        F_flag=1;
        F_state=F_state_Last;
    }
    else
    {
        F_flag=1;
        F_state=F_state_Last;        
    }
     
    F_state_Last=F_state;
    
    
}

int f_state_check_1(void)
{
    int f[7]={0,0,0,0,0,0,0};
    int temp_buf[7]={0,0,0,0,0,0,0};
    int nf=0;
    int i=0,j=0;
    int total_weight=0;
    int sum=0;
    samp_adc_1();
    
    for(i=0;i<7;i++)
    {
        if(gd_f[i]<gd_threshold_f[i])  
        {   
            f[i]=1;
            nf++;
        }
        if(f[i]==1)
        {
            temp_buf[j]=i;
            j++;
        }
    }
    F_num=nf;
    j=0;
    
    for(i=0;i<7;i++)
    {
        F_sign[i]=f[i];
    }
    
    if(nf==0) 
    {
        F_flag=1;
        if(F_state_Last<11&&F_state_Last>-11)
        {
            if(F_state_Last<0)      F_state=F_state_Last;
            else if(F_state_Last>0) F_state=F_state_Last;
            else    F_state=F_state_Last;
        }
        else
        {
            if(F_state_Last<0)  F_state=-13;
            else if(F_state_Last>0) F_state=13;
            else    F_state=F_state_Last;
        }
    }
    else if(nf==1)
    {   
        for(i=0;i<7;i++)
            total_weight=total_weight+f[i]*weight[i];
        F_state=total_weight/nf;
        F_flag=0;
    }
    else if(nf==2)
    {
        if((temp_buf[1]-temp_buf[0]==1)||(temp_buf[1]-temp_buf[0]==-1))
        {
            F_flag=0;
            for(i=0;i<7;i++)
                total_weight=total_weight+f[i]*weight[i];
            F_state=total_weight/nf;
        }
        else if(f[3]==1)
        {
            F_flag=0;
            F_state=0;
        }
        else if(abs(temp_buf[1]-3)<=abs(temp_buf[0]-3))
        {
            F_flag=1;
            F_state=weight[temp_buf[1]];
        }
        else if(abs(temp_buf[1]-3)> abs(temp_buf[0]-3))
        {
            F_flag=1;
            F_state=weight[temp_buf[0]];    
        }
        else
        {
            F_flag=1;
            for(i=0;i<7;i++)
                total_weight=total_weight+f[i]*weight[i];
            F_state=total_weight/nf;
        }
    }
    else if(nf==3)
    {
        if(temp_buf[1]==3)
        {
           F_flag=0; 
           F_state=0;
        }
        else if(f[3]==1)
        {
           F_flag=0; 
           F_state=0;
        }
        else if((temp_buf[1]-temp_buf[0]==1)&&(temp_buf[2]-temp_buf[1]==1))
        {
            F_flag=0;
            for(i=0;i<7;i++)
                total_weight=total_weight+f[i]*weight[i];
            F_state=total_weight/nf;
        }
        else
        {
            F_flag=1;
            for(i=0;i<7;i++)
                total_weight=total_weight+f[i]*weight[i];
            F_state=total_weight/nf;
        }
    }
    else if(nf==4)
    {
        F_flag=1;
        F_state=F_state_Last;
    }
    else
    {
        F_flag=1;
        F_state=F_state_Last;        
    }
     
    F_state_Last=F_state;
    
    
}
//====================================================================================//
int b_state_check_1()
{
    int b[7]={0,0,0,0,0,0,0};
    int temp_buf[7]={0,0,0,0,0,0,0};
    int nb=0;
    int i=0,j=0;
    int total_weight=0;
    int sum=0;
    samp_adc_1();
    
    for(i=0;i<7;i++)
    {
        if(gd_b[i]<gd_threshold_b[i])  
        {   
            b[i]=1;
            nb++;
        }
        if(b[i]==1)
        {
            temp_buf[j]=i;
            j++;
        }
    }
    B_num=nb;
    j=0;
    
    for(i=0;i<7;i++)
    {
        B_sign[i]=b[i];
    }
    
    if(nb==0) 
    {
        B_flag=1;
        if(B_state_Last<11&&B_state_Last>-11)
        {
            if(B_state_Last<0)  B_state=B_state_Last;
            else if(B_state_Last>0) B_state=B_state_Last;
            else    B_state=B_state_Last;
        }
        else
        {
            if(B_state_Last<0)  B_state=-13;
            else if(B_state_Last>0) B_state=13;
            else    B_state=B_state_Last;
        }
    }
    else if(nb==1)
    {   
        for(i=0;i<7;i++)
            total_weight=total_weight+b[i]*weight[i];
        B_state=total_weight/nb;
        B_flag=0;
    }
    
    else if(nb==2)
    {
        if((temp_buf[1]-temp_buf[0]==1)||(temp_buf[1]-temp_buf[0]==-1))
        {
            B_flag=0;
            for(i=0;i<7;i++)
                total_weight=total_weight+b[i]*weight[i];
            B_state=total_weight/nb;
        }
        else if(b[3]==1)
        {
            B_flag=0;
            B_state=0;
        }
        else if(abs(temp_buf[1]-3)<=abs(temp_buf[0]-3))
        {
            B_flag=1;
            B_state=weight[temp_buf[1]];
        }
        else if(abs(temp_buf[1]-3)> abs(temp_buf[0]-3))
        {
            B_flag=1;
            B_state=weight[temp_buf[0]];    
        }
        else
        {
            B_flag=1;
            for(i=0;i<7;i++)
                total_weight=total_weight+b[i]*weight[i];
            B_state=total_weight/nb;
        }
    }
    else if(nb==3)
    {
        if(temp_buf[1]==3)
        {
           B_flag=0; 
           B_state=0;
        }
        else if(b[3]==1)
        {
           B_flag=0; 
           B_state=0;
        }
        else if((temp_buf[1]-temp_buf[0]==1)&&(temp_buf[2]-temp_buf[1]==1))
        {
            B_flag=0;
            for(i=0;i<7;i++)
                total_weight=total_weight+b[i]*weight[i];
            B_state=total_weight/nb;
        }
        else
        {
            B_flag=1;
            for(i=0;i<7;i++)
                total_weight=total_weight+b[i]*weight[i];
            B_state=total_weight/nb;
        }
    }
    else if(nb==4)
    {
        B_flag=1;
        B_state=B_state_Last;
    }
    else
    {
        B_flag=1;
        B_state=B_state_Last;        
    }
       
    B_state_Last=B_state;
}

int b_state_check()
{
    int b[7]={0,0,0,0,0,0,0};
    int temp_buf[7]={0,0,0,0,0,0,0};
    int nb=0;
    int i=0,j=0;
    int total_weight=0;
    int sum=0;
    samp_adc();
    
    for(i=0;i<7;i++)
    {
        if(gd_b[i]<gd_threshold_b[i])  
        {   
            b[i]=1;
            nb++;
        }
        if(b[i]==1)
        {
            temp_buf[j]=i;
            j++;
        }
    }
    B_num=nb;
    j=0;
    
    for(i=0;i<7;i++)
    {
        B_sign[i]=b[i];
    }
    
    if(nb==0) 
    {
        B_flag=1;
        if(B_state_Last<11&&B_state_Last>-11)
        {
            if(B_state_Last<0)  B_state=B_state_Last;
            else if(B_state_Last>0) B_state=B_state_Last;
            else    B_state=B_state_Last;
        }
        else
        {
            if(B_state_Last<0)  B_state=-13;
            else if(B_state_Last>0) B_state=13;
            else    B_state=B_state_Last;
        }
    }
    else if(nb==1)
    {   
        for(i=0;i<7;i++)
            total_weight=total_weight+b[i]*weight[i];
        B_state=total_weight/nb;
        B_flag=0;
    }
    
    else if(nb==2)
    {
        if((temp_buf[1]-temp_buf[0]==1)||(temp_buf[1]-temp_buf[0]==-1))
        {
            B_flag=0;
            for(i=0;i<7;i++)
                total_weight=total_weight+b[i]*weight[i];
            B_state=total_weight/nb;
        }
        else if(b[3]==1)
        {
            B_flag=0;
            B_state=0;
        }
        else if(abs(temp_buf[1]-3)<=abs(temp_buf[0]-3))
        {
            B_flag=1;
            B_state=weight[temp_buf[1]];
        }
        else if(abs(temp_buf[1]-3)> abs(temp_buf[0]-3))
        {
            B_flag=1;
            B_state=weight[temp_buf[0]];    
        }
        else
        {
            B_flag=1;
            for(i=0;i<7;i++)
                total_weight=total_weight+b[i]*weight[i];
            B_state=total_weight/nb;
        }
    }
    else if(nb==3)
    {
        if(temp_buf[1]==3)
        {
           B_flag=0; 
           B_state=0;
        }
        else if(b[3]==1)
        {
           B_flag=0; 
           B_state=0;
        }
        else if((temp_buf[1]-temp_buf[0]==1)&&(temp_buf[2]-temp_buf[1]==1))
        {
            B_flag=0;
            for(i=0;i<7;i++)
                total_weight=total_weight+b[i]*weight[i];
            B_state=total_weight/nb;
        }
        else
        {
            B_flag=1;
            for(i=0;i<7;i++)
                total_weight=total_weight+b[i]*weight[i];
            B_state=total_weight/nb;
        }
    }
    else if(nb==4)
    {
        B_flag=1;
        B_state=B_state_Last;
    }
    else
    {
        B_flag=1;
        B_state=B_state_Last;        
    }
       
    B_state_Last=B_state;
}
//==================================================================================//
void test_adc()
{
    
    while(1)
    { 
        samp_adc();
        cls();
        printf("f0=%3d f1=%3d f2=%3d",gd_f[0],gd_f[1],gd_f[2]);
        locate(2,1);
        printf("f3=%3d f4=%3d f5=%3d",gd_f[3],gd_f[4],gd_f[5]);
        locate(3,1);
        printf("f6=%3d l=%3d r=%3d",gd_f[6],gd_l,gd_r);
        locate(4,1);
        printf("b0=%3d b1=%3d b2=%3d",gd_b[0],gd_b[1],gd_b[2]);
        locate(5,1);
        printf("b3=%3d b4=%3d b5=%3d",gd_b[3],gd_b[4],gd_b[5]);
        locate(6,1);
        printf("b6=%3d fa=%3d ba=%3d",gd_b[6],r_af,r_ab);
        wait(0.1);
        
    }
}
void test_port()
{
    while(1)
    { 
        samp_port();
        cls();
        locate(1,1); printf("t_fl=%d t_fr=%d",t_fl,t_fr);    //t_fl,t_fr,t_bl,t_br
        locate(2,1); printf("r_xf=%d r_xb=%d",r_xf,r_xb);
        locate(3,1); printf("r_df=%d r_db=%d r_fh=%d",r_df,r_db,r_fh);
        locate(4,1); printf("t_bl=%d t_br=%d",t_bl,t_br);
        locate(5,1); printf("led=%d ",led);
        locate(6,1); printf("com=%d ",com);
        wait(0.1);
    }
}
void test_port2()
{
    while(1)
    { 
        samp_port2();
        cls();
        locate(1,1); printf("t_fl=%d t_fr=%d",t_fl,t_fr);    //t_fl,t_fr,t_bl,t_br
        locate(2,1); printf("r_xf=%d r_xb=%d",r_xf,r_xb);
        locate(3,1); printf("r_df=%d r_db=%d r_fh=%d",r_df,r_db,r_fh);
        locate(4,1); printf("t_bl=%d t_br=%d",t_bl,t_br);
        locate(5,1); printf("led=%d ",led);
        locate(6,1); printf("com=%d ",com);
        wait(0.1);
    }
}
void go_front(int set_speed)
{
    f_state_check();
    b_state_check();
    int right_speed;
    int left_speed;
    int error;
    if(B_flag==1)   error=3*F_state;    
    else            error=2*F_state+1*(F_state-B_state);
    int kp=40;    //40
    left_speed=set_speed+error*kp/10;        
    right_speed=set_speed-error*kp/10;
    
    if(left_speed>1000)     left_speed=1000;
    else if(left_speed<0)   left_speed=0;
    else;
    if(right_speed>1000)    right_speed=1000;
    else if(right_speed<0)   right_speed=0;
    else; 
    motor(1,left_speed);motor(2,right_speed);
    wait(0.001);
}

void go_front_1(int set_speed)
{
    f_state_check_1();
    b_state_check_1();
    int right_speed;
    int left_speed;
    int error;
    if(B_flag==1)   error=3*F_state;    
    else            error=2*F_state+1*(F_state-B_state);
    int kp=40;    //40
    left_speed=set_speed+error*kp/10;        
    right_speed=set_speed-error*kp/10;
    
    if(left_speed>1000)     left_speed=1000;
    else if(left_speed<0)   left_speed=0;
    else;
    if(right_speed>1000)    right_speed=1000;
    else if(right_speed<0)   right_speed=0;
    else; 
    motor(1,left_speed);motor(2,right_speed);
    wait(0.001);
}
//====================================================================================//
void go_back( int set_speed)
{
    f_state_check();
    b_state_check();
    int right_speed;
    int left_speed;
    int error;
    if(F_flag==1)   error=3*B_state;
    else            error=2*B_state+1*(B_state-F_state);
    int kp=40;
    left_speed=-set_speed-error*kp/10;
    right_speed=-set_speed+error*kp/10;
    
    if(left_speed<-1000)     left_speed=-1000;
    else if(left_speed>0)   left_speed=0;
    else;
    if(right_speed<-1000)    right_speed=-1000;
    else if(right_speed>0)   right_speed=0;
    else; 
    motor(1,left_speed);motor(2,right_speed);
    wait(0.001);
}

void go_front_b( int set_speed)
{
    f_state_check();
    b_state_check();
    int right_speed;
    int left_speed;
    int error;
    if(F_flag==1)   error=3*B_state;
    else            error=2*B_state+1*(B_state-F_state);
    int kp=120;
    left_speed=-(-set_speed-error*kp/10);
    right_speed=-(-set_speed+error*kp/10);
    
    if(left_speed>1000)     left_speed=1000;
    else if(left_speed<0)   left_speed=0;
    else;
    if(right_speed>1000)    right_speed=1000;
    else if(right_speed<0)   right_speed=0;
    else; 
    motor(1,left_speed);motor(2,right_speed);
    wait(0.001);
}
//====================================================================================//
//====================================================================================//
void go_front_s(int set_speed)
{
    f_state_check();
    b_state_check();
    int right_speed;
    int left_speed;
    int error;
    error=4.5*F_state;    //3
    int kp=80;    //40   ;

    left_speed=set_speed+error*kp/10;        
    right_speed=set_speed-error*kp/10;
    
    if(left_speed>1000)     left_speed=1000;
    else if(left_speed<0)   left_speed=0;
    else;
    if(right_speed>1000)    right_speed=1000;
    else if(right_speed<0)   right_speed=0;
    else; 
    motor(1,left_speed);motor(2,right_speed);
    //wait(0.001);
}
//====================================================================================//
void go_back_s(set_speed)
{
    f_state_check();
    b_state_check();
    int right_speed;
    int left_speed;
    int error=0;
    error=3*B_state;
    int kp=40;
    if(set_speed<150)   kp=20;
    else if(set_speed<250)  kp=30;
    else if(set_speed<320)  kp=40;
    else if(set_speed<380)  kp=45;
    else if(set_speed<420)  kp=50;
    else      ;
    left_speed=-set_speed-error*kp/10;
    right_speed=-set_speed+error*kp/10;
    
    if(left_speed<-1000)     left_speed=-1000;
    else if(left_speed>0)   left_speed=0;
    else;
    if(right_speed<-1000)    right_speed=-1000;
    else if(right_speed>0)   right_speed=0;
    else; 
    motor(1,left_speed);motor(2,right_speed);
    //wait(0.001);
}
//====================================================================================//
void go_round_f(int set_speed,int type)  
{         //0逆时针；1顺时针
    f_state_check();
    b_state_check();
    if(set_speed<250)
    {
        if(type==0)
        {
            F_state=F_state-3;
            B_state=B_state-3;
        }
        else
        {
            F_state=F_state+3;
            B_state=B_state+3;
        } 
    }
    else
    {
        if(type==0)
        {
            F_state=F_state-5;
            B_state=B_state-5;
        }
        else
        {
            F_state=F_state+5;
            B_state=B_state+5;
        } 
    }
    int right_speed;
    int left_speed;
    int error;
    if(B_flag==1)   error=3*F_state;    
    else            error=2*F_state+1*(F_state-B_state);
    int kp=60;    //40
    if(type==0)
    {
        left_speed=set_speed+error*kp/10-60;        
        right_speed=set_speed-error*kp/10+60;
    }
    else
    {
        left_speed=set_speed+error*kp/10+60;        
        right_speed=set_speed-error*kp/10-60;
    }
    if(left_speed>1000)     left_speed=1000;
    else if(left_speed<0)   left_speed=0;
    else;
    if(right_speed>1000)    right_speed=1000;
    else if(right_speed<0)   right_speed=0;
    else; 
    motor(1,left_speed);motor(2,right_speed);
}

//====================================================================================//

void go_round_b(int set_speed,int type)        //0逆时针；1顺时针//
{
    f_state_check();
    b_state_check();
    if(set_speed<250)
    {
        if(type==0)
        {
            F_state=F_state+2;
            B_state=B_state+2;
        }
        else
        {
            F_state=F_state-2;
            B_state=B_state-2;
        }
    } 
    else   
    {
        if(type==0)
        {
            F_state=F_state+4;
            B_state=B_state+4;
        }
        else
        {
            F_state=F_state-4;
            B_state=B_state-4;
        }
    } 
    int right_speed;
    int left_speed;
    int error=0;
    if(F_flag==1)   error=3*B_state;
    else            error=2*B_state+1*(B_state-F_state);
    int kp=60;

    if(type==0)
    {
        left_speed=-set_speed-error*kp/10-60;
        right_speed=-set_speed+error*kp/10+60;
    }
    else
    {
        left_speed=-set_speed-error*kp/10+60;
        right_speed=-set_speed+error*kp/10-60;
    }    
    
    if(left_speed<-1000)     left_speed=-1000;
    else if(left_speed>0)   left_speed=0;
    else;
    if(right_speed<-1000)    right_speed=-1000;
    else if(right_speed>0)   right_speed=0;
    else; 
    motor(1,left_speed);motor(2,right_speed);
}

//====================================================================================//
void turn_fr(float t0)//前右转
{ int m=230;
int n=-230;
    samp();
    tt=seconds(1)+0.05;
    while(seconds(1)<tt){motor(1,m);motor(2,n);samp();}
    tt=seconds(1)+t0;
    while(seconds(1)<tt&&gd_f[6]<gd_threshold_f[6]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_f[5]<gd_threshold_f[5]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_f[4]<gd_threshold_f[4]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_f[3]<gd_threshold_f[3]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_f[6]>gd_threshold_f[6]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_f[5]>gd_threshold_f[5]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_f[4]>gd_threshold_f[4]&&gd_f[3]>gd_threshold_f[3]){motor(1,m);motor(2,n);samp();}
    motor(1,0);motor(2,0);
}

void turn_fl(float t0)//前左转
{int m=-230;
int n=230;
    samp();
        tt=seconds(1)+0.05;
    while(seconds(1)<tt){motor(1,-160);motor(2,160);samp();}
    tt=seconds(1)+t0;
    while(seconds(1)<tt&&gd_f[0]<gd_threshold_f[0]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_f[1]<gd_threshold_f[1]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_f[2]<gd_threshold_f[2]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_f[3]<gd_threshold_f[3]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_f[0]>gd_threshold_f[0]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_f[1]>gd_threshold_f[1]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_f[2]>gd_threshold_f[2]&&gd_f[3]>gd_threshold_f[3]){motor(1,m);motor(2,n);samp();}
    motor(1,0);motor(2,0);
    

}

void turn_br(float t0)//后右转
{ int m=230;
int n=-230;
    samp();
        tt=seconds(1)+0.05;
    while(seconds(1)<tt){motor(1,m);motor(2,n);samp();}
    tt=seconds(1)+t0;
    while(seconds(1)<tt&&gd_b[0]<gd_threshold_b[0]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_b[1]<gd_threshold_b[1]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_b[2]<gd_threshold_b[2]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_b[3]<gd_threshold_b[3]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_b[0]>gd_threshold_b[0]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_b[1]>gd_threshold_b[1]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_b[2]>gd_threshold_b[2]&&gd_b[3]>gd_threshold_b[3]){motor(1,m);motor(2,n);samp();}
    motor(1,0);motor(2,0);

}

void turn_bl(float t0)//后左转
{ int m=-230;
int n=230;
    samp();
        tt=seconds(1)+0.05;
    while(seconds(1)<tt){motor(1,m);motor(2,n);samp();}
    tt=seconds(1)+t0;
    while(seconds(1)<tt&&gd_b[6]<gd_threshold_b[6]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_b[5]<gd_threshold_b[5]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_b[4]<gd_threshold_b[4]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_b[3]<gd_threshold_b[3]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_b[6]>gd_threshold_b[6]){motor(1,m);motor(2,n);samp();}
    while(seconds(1)<tt&&gd_b[5]>gd_threshold_b[5]){motor(1,m);motor(2,n);samp();} 
    while(seconds(1)<tt&&gd_b[4]>gd_threshold_b[4]&&gd_b[3]>gd_threshold_b[3]){motor(1,m);motor(2,n);samp();}  
    motor(1,0);motor(2,0);
}
//=================================================================================//
void turn(int left_speed,int right_speed,int type)//"type" should be one of "1,2,3,4";1 is fr;2 fl;3 br;4 bl;
{
    if(type==1)
    {
        samp();
        tt=seconds(1)+0.6;
        while(seconds(1)<tt&&gd_f[6]<gd_threshold_f[6]){samp();go2(left_speed,-right_speed);}
        while(seconds(1)<tt&&gd_f[5]<gd_threshold_f[5]){samp();go2(left_speed,-right_speed);}
        while(seconds(1)<tt&&gd_f[4]<gd_threshold_f[4]){samp();go2(left_speed,-right_speed);}
        while(seconds(1)<tt&&gd_f[3]<gd_threshold_f[3]){samp();go2(left_speed,-right_speed);}
        while(seconds(1)<tt&&gd_f[6]>gd_threshold_f[6]){samp();go2(left_speed,-right_speed);}
        while(seconds(1)<tt&&gd_f[5]>gd_threshold_f[5]){samp();go2(left_speed,-right_speed);}
        while(seconds(1)<tt&&gd_f[4]>gd_threshold_f[4]){samp();go2(left_speed,-right_speed);}
        while(seconds(1)<tt&&gd_f[3]>gd_threshold_f[3]){samp();go2(left_speed,-right_speed);}
        motor(1,0);motor(2,0);
    }
    else if(type==2)
    {
        samp();
        tt=seconds(1)+0.6;
        while(seconds(1)<tt&&gd_f[0]<gd_threshold_f[0]){samp();go2(-left_speed,right_speed);}
        while(seconds(1)<tt&&gd_f[1]<gd_threshold_f[1]){samp();go2(-left_speed,right_speed);}
        while(seconds(1)<tt&&gd_f[2]<gd_threshold_f[2]){samp();go2(-left_speed,right_speed);}
        while(seconds(1)<tt&&gd_f[3]<gd_threshold_f[3]){samp();go2(-left_speed,right_speed);}
        while(seconds(1)<tt&&gd_f[0]>gd_threshold_f[0]){samp();go2(-left_speed,right_speed);}
        while(seconds(1)<tt&&gd_f[1]>gd_threshold_f[1]){samp();go2(-left_speed,right_speed);}
        while(seconds(1)<tt&&gd_f[2]>gd_threshold_f[2]&&gd_f[3]>gd_threshold_f[3]){samp();go2(-left_speed,right_speed);}
        motor(1,0);motor(2,0);
    }
    else if(type==3)
    {
        samp();
        tt=seconds(1)+0.6;
        while(seconds(1)<tt&&gd_b[0]<gd_threshold_b[0]){samp();go2(left_speed,-right_speed);}
        while(seconds(1)<tt&&gd_b[1]<gd_threshold_b[1]){samp();go2(left_speed,-right_speed);}
        while(seconds(1)<tt&&gd_b[2]<gd_threshold_b[2]){samp();go2(left_speed,-right_speed);}
        while(seconds(1)<tt&&gd_b[3]<gd_threshold_b[3]){samp();go2(left_speed,-right_speed);}
        while(seconds(1)<tt&&gd_b[0]>gd_threshold_b[0]){samp();go2(left_speed,-right_speed);}
        while(seconds(1)<tt&&gd_b[1]>gd_threshold_b[1]){samp();go2(left_speed,-right_speed);}
        while(seconds(1)<tt&&gd_b[2]>gd_threshold_b[2]&&gd_b[3]>gd_threshold_b[3]){samp();go2(left_speed,-right_speed);}
        motor(1,0);motor(2,0);
    }
    else if(type==4)
    {
        samp();
        tt=seconds(1)+0.6;
        while(seconds(1)<tt&&gd_b[6]<gd_threshold_b[6]){samp();go2(-left_speed,right_speed);}
        while(seconds(1)<tt&&gd_b[5]<gd_threshold_b[5]){samp();go2(-left_speed,right_speed);}
        while(seconds(1)<tt&&gd_b[4]<gd_threshold_b[4]){samp();go2(-left_speed,right_speed);}
        while(seconds(1)<tt&&gd_b[3]<gd_threshold_b[3]){samp();go2(-left_speed,right_speed);}
        while(seconds(1)<tt&&gd_b[6]>gd_threshold_b[6]){samp();go2(-left_speed,right_speed);}
        while(seconds(1)<tt&&gd_b[5]>gd_threshold_b[5]){samp();go2(-left_speed,right_speed);}
        while(seconds(1)<tt&&gd_b[4]>gd_threshold_b[4]&&gd_b[3]>gd_threshold_b[3]){samp();go2(-left_speed,right_speed);}    
        motor(1,0);motor(2,0);
    }    
}
//=================================================================================//
void soft_start_f()    
{        //缓慢向前启动
    tt=seconds(1)+0.01;
    while(tt>seconds(1))    go_front(100);
    tt=seconds(1)+0.01;
    while(tt>seconds(1))    go_front(150);
    tt=seconds(1)+0.01;
    while(tt>seconds(1))   { go_front(200);sound(500,50);}
    
}
void soft_start_b()            
{                     //缓慢向后启动
    tt=seconds(1)+0.03;
    while(tt>seconds(1))    go_back(100);
   // tt=seconds(1)+0.01;
   // while(tt>seconds(1))    go_back(150);
   // tt=seconds(1)+0.01;
   // while(tt>seconds(1))    go_back(200);
    
}
//================================================================================//
void soft_land_f(void)              
{                  //缓慢向前停止
  
    tt=seconds(1)+0.01;
    while(tt>seconds(1))    go_front(200);
    tt=seconds(1)+0.01;
    while(tt>seconds(1))    go_front(150);
    tt=seconds(1)+0.01;
    while(tt>seconds(1))    go_front(100);
    
}
void soft_land_turn(fast,slow)              
{                  //缓慢向前停止
  
    tt=seconds(1)+0.01;
    while(tt>seconds(1))    go_front(fast);
    tt=seconds(1)+0.01;
    while(tt>seconds(1))    go_front((fast+slow)/2);
    tt=seconds(1)+0.01;
    while(tt>seconds(1))    go_front(slow);
    
}
void soft_land_b(void)               
{                     //缓慢向后停止
   
    tt=seconds(1)+0.01;
    while(tt>seconds(1))    go_back(200);
    tt=seconds(1)+0.01;
    while(tt>seconds(1))    go_back(150);
    tt=seconds(1)+0.01;
    while(tt>seconds(1))    go_back(100);
   
    
}
//======================================================================================//
void f_bump(int fast_speed,int slow_speed)        
{                                                  //碰撞bump
    samp();
    
    while(r_df==0&&t_fl==0&&t_fr==0)
    {
        go_front(fast_speed);
        samp();
    }
    while(t_fl==0&&t_fr==0)
    {
        go_front(slow_speed);
        samp();
    }
    motor(1,0);motor(2,0);
}
void b_bump(int fast_speed,int slow_speed)         
{                                                //碰撞bump
    samp();
    tt=seconds(1)+0.1;
    while(seconds(1)<tt){go_back(170);samp();}
    while(r_db==0&&t_bl==0&&t_br==0)
    {
        go_back(fast_speed);
        samp();
    }
    while(t_bl==0&&t_br==0)
    {
        go_back(slow_speed);
        samp();
    }
    motor(1,0);motor(2,0);
}



void comturn()
{
   int a;
   a=compass()+180;
   locate(1,1); printf("compass=%d",compass());
   if (a<360)
   { while (compass()<a) {motor(1,300);motor(2,-300);}
     motor(1,0);motor(2,0);wait(0.2);
    }
   else
   { while ((180<compass())&&(compass()<360)) {motor(1,300);motor(2,-300);}
     while (compass()<(a-360)) {motor(1,300);motor(2,-300);}
   }
    motor(1,0);motor(2,0);wait(0.2);
       locate(2,1); printf("compass=%d",compass());
}

void comturn2()
{
   int a;
   locate(1,1); printf("compass=%d",compass());
   a=compass()+180;
   if (a>360) a=a-360;
   while ((compass()<a-1)||(compass()>a+1)) {motor(1,300);motor(2,-300);samp();}
   motor(1,0);motor(2,0);wait(0.2);
      locate(2,1); printf("compass=%d",compass());
}

void comturn3(int round)
{
samp();
    com1=com;
    com1=com1+round;
    if(com1>359)com1=com1-360;
    tt=seconds(1)+0.1;
    while(seconds(1)<tt){go2(250,-250);samp();}
    while(1)
    {
        samp();
      com2=6*(com-com1);
        if(com2<0)com2=-com2;
        while(com2>300)
        {
        go2(250,-250);
        samp();
        com2=6*(com-com1);
        if(com2<0)com2=-com2;
        }
        go2(com2,-com2);
        while(com2<70)
        {
        go2(120,-120);
        samp();
        com2=6*(com-com1);
        if(com2<0)com2=-com2;
        if(com==com1)break;
        }
        if(com==com1)break;
    }
    motor(1,0);motor(2,0);
    wait(1);
}
void comturn4()
{
   int a;
   a=compass()+180;
   locate(1,1); printf("compass=%d",compass());
   if (a>=180&&a<270)
   { if(compass()<=6&&compass()>=0)
     { tt=seconds(1)+0.06;
       while(tt>seconds(1)) {motor(1,220);motor(2,-220);}
     }
     while (compass()<a-60) {motor(1,220);motor(2,-220);}
     if(compass()<=6&&compass()>=0) 
          while (compass()<a-2) {motor(1,130);motor(2,-130);}
     else
          while (compass()<a-1) {motor(1,130);motor(2,-130);}
     motor(1,0);motor(2,0);wait(0.2);
    }

   if (a>=270&&a<360)
   {
     while (compass()<a-60) {motor(1,220);motor(2,-220);}
     if(a-2<360)
     while (compass()<a-2) {motor(1,130);motor(2,-130);}
     else
     {
       while (compass()<360) {motor(1,130);motor(2,-130);}
       while (compass()<a-360-2) {motor(1,130);motor(2,-130);}
      }
     motor(1,0);motor(2,0);wait(0.2);
    }

   if (a>=360&&a<450)
   {
      if(a-60<360)
     { while (compass()<a-60) {motor(1,220);motor(2,-220);}
       while (compass()<360&&compass()>180) {motor(1,130);motor(2,-130);}
       while (compass()<a-360-1) {motor(1,130);motor(2,-130);}
       motor(1,0);motor(2,0);wait(0.2);
     }
      else
     { 
       while (compass()<360&&compass()>180) {motor(1,220);motor(2,-220);}
       while (compass()<a-360-60) {motor(1,220);motor(2,-220);}
       while (compass()<a-360-1) {motor(1,130);motor(2,-130);}
       motor(1,0);motor(2,0);wait(0.2);
      }
   }

   if (a>=450&&a<540)
   {
       while (compass()<360&&compass()>180) {motor(1,220);motor(2,-220);}
       while (compass()<a-360-60) {motor(1,220);motor(2,-220);}
       while (compass()<a-360-2) {motor(1,130);motor(2,-130);}
       motor(1,0);motor(2,0);wait(0.2);
    }
    locate(2,1); printf("compass=%d",compass());
} 


void test_compass()
{  int fast_speed=250;
   int slow_speed=150;
//=======================速度设定
   int a;
   a=compass()+180;
   cls();
   locate(1,1); printf("compass=%d",compass());
   if (a>=180&&a<270)
   { if(a>=180&&a<186)
      {  tt=seconds(1)+0.5;
         while(tt>seconds(1)) {motor(1,fast_speed);motor(2,-fast_speed);}
       }
     while (compass()<a-60) {motor(1,fast_speed);motor(2,-fast_speed);}
     while (compass()<a) {motor(1,slow_speed);motor(2,-slow_speed);}
     motor(1,0);motor(2,0);wait(0.2);
    }

   if (a>=270&&a<360)
   {
     while (compass()<a-60) {motor(1,fast_speed);motor(2,-fast_speed);}
     while (compass()<a) {motor(1,slow_speed);motor(2,-slow_speed);}
     motor(1,0);motor(2,0);wait(0.2);
    }

   if (a>=360&&a<450)
   {
      if(a-60<360)
     { while (compass()<a-60) {motor(1,fast_speed);motor(2,-fast_speed);}
       while (compass()<360&&compass()>180) {motor(1,slow_speed);motor(2,-slow_speed);}
       while (compass()<a-360) {motor(1,slow_speed);motor(2,-slow_speed);}
       motor(1,0);motor(2,0);wait(0.2);
     }
      else
     { 
       while (compass()<360&&compass()>180) {motor(1,fast_speed);motor(2,-fast_speed);}
       while (compass()<a-360-60) {motor(1,fast_speed);motor(2,-fast_speed);}
       while (compass()<a-360) {motor(1,slow_speed);motor(2,-slow_speed);}
       motor(1,0);motor(2,0);wait(0.2);
      }
   }

   if (a>=450&&a<540)
   {
       while(compass()<360&&compass()>180) {motor(1,fast_speed);motor(2,-fast_speed);}
       while(compass()<a-360-60) {motor(1,fast_speed);motor(2,-fast_speed);}
       while(compass()<a-360)  {motor(1,slow_speed);motor(2,-slow_speed);}
    }

   locate(2,1); printf("compass=%d",compass());
}


void comturn_180()
{
//=======================速度设定
   int fast_speed=250;
   int slow_speed=130;
//========================旋转偏移角测值补偿
   int pan_0=0;
   int pan_45=0;
   int pan_90=0;
   int pan_135=0;
   int pan_180=0;
   int pan_225=0;
   int pan_270=0;
   int pan_315=0;
//========================各区间单位偏移角
   float pan_0_45=1/45*( pan_45-pan_0);
   float pan_45_90=1/45*( pan_90-pan_45);
   float pan_90_135=1/45*( pan_135-pan_90);
   float pan_135_180=1/45*( pan_180-pan_135);
   float pan_180_225=1/45*( pan_225-pan_180);
   float pan_225_270=1/45*( pan_270-pan_225);
   float pan_270_315=1/45*( pan_315-pan_270);
   float pan_315_360=1/45*( pan_0-pan_315);

   int a,seta;
   seta=compass();
   a=seta+180;

   cls();
   locate(1,1); printf("compass=%d",seta);
   tt=seconds(1)+0.5;
   while(tt>seconds(1)) {motor(1,fast_speed);motor(2,-fast_speed);} 
   if(seta>=0&&seta<180)
  {  
     while (compass()<a-60) {motor(1,fast_speed);motor(2,-fast_speed);}
     if(seta>=0&&seta<45)
        while(compass()<a+pan_0+(seta-0)*pan_0_45) {motor(1,slow_speed);motor(2,-slow_speed);}
     if(seta>=45&&seta<90)
        while(compass()<a+pan_45+(seta-45)*pan_45_90) {motor(1,slow_speed);motor(2,-slow_speed);}
     if(seta>=90&&seta<135)
        while(compass()<a+pan_90+(seta-90)*pan_90_135) {motor(1,slow_speed);motor(2,-slow_speed);}
     if(seta>=135&&seta<180)
         if(a+pan_135+(seta-135)*pan_135_180<360)
            while(compass()<a+pan_135+(seta-135)*pan_135_180) {motor(1,slow_speed);motor(2,-slow_speed);}
         else
           {  while(compass()<360&&compass()>180) {motor(1,slow_speed);motor(2,-slow_speed);} 
              while(compass()<a-360+pan_135+(seta-135)*pan_135_180) {motor(1,slow_speed);motor(2,-slow_speed);} 
           }
     motor(1,0);motor(2,0);
   }

   if(seta>=180&&seta<360)
  {       
      if(seta>=180&&seta<225) 
         { while (compass()<a-60) {motor(1,fast_speed);motor(2,-fast_speed);}
           if(a+pan_180+(seta-180)*pan_180_225<360)
               while(compass()<a+pan_180+(seta-180)*pan_180_225) {motor(1,slow_speed);motor(2,-slow_speed);}
           else
              { while(compass()<360&&compass()>180) {motor(1,slow_speed);motor(2,-slow_speed);}
                while(compass()<a-360+pan_180+(seta-180)*pan_180_225) {motor(1,slow_speed);motor(2,-slow_speed);}
               }
          }
      if(seta>=225&&seta<270) 
         {
           if(a-60<360)
               { while(compass()<a-90) {motor(1,fast_speed);motor(2,-fast_speed);}
                 while(compass()<360&&compass()>180) {motor(1,slow_speed);motor(2,-slow_speed);}
                }
           else if(a-60>360)
               { while(compass()<360&&compass()>180) {motor(1,fast_speed);motor(2,-fast_speed);}
                 while(compass()<a-360-60) {motor(1,fast_speed);motor(2,-fast_speed);}
                }
           else{
                 while(compass()<360&&compass()>180) {motor(1,fast_speed);motor(2,-fast_speed);}
                 tt=seconds(1)+0.2;
                 while(tt>seconds(1)) {motor(1,slow_speed);motor(2,-slow_speed);} 
                }
           while(compass()<a-360+pan_225+(seta-225)*pan_225_270) {motor(1,slow_speed);motor(2,-slow_speed);}
          }

      if(seta>=270&&seta<315) 
          {  while(compass()<360&&compass()>180) {motor(1,fast_speed);motor(2,-fast_speed);}
             while(compass()<a-360-60) {motor(1,fast_speed);motor(2,-fast_speed);}
             while(compass()<a-360+pan_270+(seta-270)*pan_270_315)  {motor(1,slow_speed);motor(2,-slow_speed);}
          }
      if(seta>=315&&seta<360) 
          {  
             while(compass()<360&&compass()>180) {motor(1,fast_speed);motor(2,-fast_speed);}
             while(compass()<a-360-60) {motor(1,fast_speed);motor(2,-fast_speed);}
             while(compass()<a-360+pan_315+(seta-315)*pan_315_360)  {motor(1,slow_speed);motor(2,-slow_speed);}
          }
      motor(1,0);motor(2,0);
   }
  
   locate(2,1); printf("compass=%d",compass());

}

void comturn_5()
{
   int fast_speed=260;
   int slow_speed=140;
   int a,seta;
   seta=compass();
   a=seta+180;
   cls();
   locate(1,1); printf("compass=%d",seta);
   tt=seconds(1)+0.3;
   while(tt>seconds(1)) {motor(1,fast_speed);motor(2,-fast_speed);} 
   if(seta>=0&&seta<180)
    {   while(compass()<a-60)
        {motor(1,fast_speed);motor(2,-fast_speed);}
        while(compass()<a)
        {motor(1,slow_speed);motor(2,-slow_speed);}
    }
    
    if(seta>=180&&seta<360)
    {   if(a-60<360)
           {  while(compass()<a-90)
              {motor(1,fast_speed);motor(2,-fast_speed);}
              while(compass()<360&&compass()>180)
              {motor(1,slow_speed);motor(2,-slow_speed);} 
            }
        else
           { while(compass()<360&&compass()>180)
             {motor(1,fast_speed);motor(2,-fast_speed);}
             while(compass()<a-60-360)
             {motor(1,fast_speed);motor(2,-fast_speed);}
            }
        while(compass()<a-360)
           {motor(1,slow_speed);motor(2,-slow_speed);}
    }
    motor(1,0);motor(2,0);
    locate(2,1); printf("compass=%d",compass());
}

void comturn_hh(int n ,int locate,int lcd,double t)
{
    go(0,0);
    wait(0.5);
    dirGoal=compass()+n;
    if(dirGoal<0)
    {
    dirGoal+=360;
    }
    else if(dirGoal>360)
    {
    dirGoal-=360;
    }
    if(n>0)
    {
    motor(1,fast_speed-30);motor(2,-(fast_speed-45));
    while(1)
        {
        dirBuff=compass();
        if(((dirGoal-2)<dirBuff)&&(dirBuff<(dirGoal+2)))
            {
            break;
            }
        }
        motor(1,0);motor(2,0);
    }
    else
    {
    motor(1,-(fast_speed-45));motor(2,fast_speed-30);
    while(1)
        {
        dirBuff=compass();
        if(((dirGoal-2)<dirBuff)&&(dirBuff<(dirGoal+2)))
            {
            break;
            }
        }
        motor(1,0);motor(2,0);
    }
}

void passdoor(void)  
{  
   soft_start_f();
   samp();
   while(!r_xb){go_front(160);samp();}
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;
   while(gd_f[0]>330)  {go_front(120);samp();}//200gaiwei120
   while(gd_l<272)  {go_front(100);samp();}//150gaiwei100
   while(gd_b[0]>335)  {go_front(100);samp();}//150gaiwei100
   motor(1,0);motor(2,0);wait(0.2);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=436; 
    gd_threshold_b[1]=430;
    gd_threshold_b[2]=433;
    gd_threshold_b[3]=416;
    gd_threshold_b[4]=433;
    gd_threshold_b[5]=413;
    gd_threshold_b[6]=424; 
    gd_threshold_l=154;
    soft_start_f();
}
void passdoor2(void)  
{  
   soft_start_b();
   samp();
      while(!r_xf){go_back(160);samp();}
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;
   while(gd_b[0]>335)  {go_back(120);samp();}//200gaiwei120
   while(gd_l<272)  {go_back(100);samp();}//150gaiwei100
   while(gd_f[0]>330)  {go_back(100);samp();}//150gaiwei100
   motor(1,0);motor(2,0);wait(0.2);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=436; 
    gd_threshold_b[1]=430;
    gd_threshold_b[2]=433;
    gd_threshold_b[3]=416;
    gd_threshold_b[4]=433;
    gd_threshold_b[5]=413;
    gd_threshold_b[6]=424; 
    gd_threshold_l=179;
}

void passbridge1(void)
{
   soft_start_f();
   samp();
   while(!r_xb) {go_front(100);samp();}
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(100);samp();}
    while(!r_xf) {go_front(120);samp();}     //斜面1
    sound(1000,100);
    motor(1,0);motor(2,0);wait(0.2);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=436; 
    gd_threshold_b[1]=430;
    gd_threshold_b[2]=433;
    gd_threshold_b[3]=416;
    gd_threshold_b[4]=433;
    gd_threshold_b[5]=413;
    gd_threshold_b[6]=424; 
    gd_threshold_l=179;
     tt=seconds(1)+0.8;
    while(seconds(1)<tt){go_front(80);samp();}
    motor(1,0);motor(2,0);wait(0.2);
    sound(1000,100);

}

void passbridge2(void)
{
   soft_start_f();
   samp();
   while(!r_xb) {go_front(100);samp();}
       gd_threshold_f[0]=374;
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 

     tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(100);samp();}

         while(!r_xf) {go_front(120);samp();}
         sound(1000,100);
         motor(1,0);motor(2,0);wait(0.2);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=436; 
    gd_threshold_b[1]=430;
    gd_threshold_b[2]=433;
    gd_threshold_b[3]=416;
    gd_threshold_b[4]=433;
    gd_threshold_b[5]=413;
    gd_threshold_b[6]=424; 
    gd_threshold_l=179;
     tt=seconds(1)+0.8;
    while(seconds(1)<tt){go_front(80);samp();}
             motor(1,0);motor(2,0);wait(0.2);
sound(1000,100);

}

void qiaobanqiao(void)
{
   soft_start_f();
   samp();
   while(!r_xb) {go_front(180);samp();}
   
    gd_threshold_f[0]=353;
    gd_threshold_f[1]=294;
    gd_threshold_f[2]=315;
    gd_threshold_f[3]=300;
    gd_threshold_f[4]=336;
    gd_threshold_f[5]=335;
    gd_threshold_f[6]=361;

    gd_threshold_b[0]=362; 
    gd_threshold_b[1]=350;
    gd_threshold_b[2]=342;
    gd_threshold_b[3]=302;
    gd_threshold_b[4]=280;
    gd_threshold_b[5]=254;
    gd_threshold_b[6]=301; 
    
    gd_threshold_l=213;
    
 
      tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_front(140);samp();}
    sound(1000,100);

   while(r_db) {go_front(120);samp();}
       motor(1,0);motor(2,0);wait(0.2);
   while(!r_xf) {go_front(100);samp();}
    motor(1,0);motor(2,0);wait(0.2);
    samp();
   // tt=seconds(1)+0.2;
   // while(seconds(1)<tt){go_front(120);samp();}
    sound(1000,100);

    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=179;
    
}


void platform()
{
int fast=200;
int slow=80;
float m=0.1;
    while(!r_df)  {go_front(fast-20);samp();}
    motor(1,0);motor(2,0);wait(m); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go_front(slow+20);samp();}
     sound(1000,1000);
     while(gd_f[0]>435){go2(150,150);samp();}
     while(gd_r>60){go2(120,120);samp();}
   while(gd_b[0]>440){go2(100,100);samp();}
  //  while(gd_l<gd_threshold_l)  {go_front(120);samp();}
  //      tt=seconds(1)+0.4;
  //  while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
    motor(1,0);motor(2,0);
wait(m);  
comturn_hh(165,-1,0,0);
    motor(1,0);motor(2,0);
    samp(); wait(m);  
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(120);samp();} 
    sound(1000,100);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=154;                  
}
//====================================================================================//

void platform2()
{
int fast=200;
int slow=80;
float m=0.1;
    while(!r_df)  {go_front(fast-20);samp();}
    motor(1,0);motor(2,0);wait(m); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go_front(slow+20);samp();}
     sound(1000,1000);
     while(gd_f[0]>435){go2(150,150);samp();}
     while(gd_r>60){go2(120,120);samp();}
   while(gd_b[0]>440){go2(100,100);samp();}
  //  while(gd_l<gd_threshold_l)  {go_front(120);samp();}
  //      tt=seconds(1)+0.4;
  //  while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
    motor(1,0);motor(2,0);
wait(m);  
comturn_hh(165,-1,0,0);
    motor(1,0);motor(2,0);
    samp(); wait(m);  
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(120);samp();} 
    sound(1000,100);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=154;               
}

void platform3()
{
int fast=220;
int slow=80;
float m=0.1;
    while(!r_df)  {go_front(fast-20);samp();}
    motor(1,0);motor(2,0);wait(m); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go_front(slow+20);samp();}
     sound(1000,1000);
     while(gd_f[0]>435){go2(150,150);samp();}
     while(gd_r>60){go2(120,120);samp();}
   while(gd_b[0]>440){go2(100,100);samp();}
  //  while(gd_l<gd_threshold_l)  {go_front(120);samp();}
  //      tt=seconds(1)+0.4;
  //  while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
    motor(1,0);motor(2,0);
wait(m);  
comturn_hh(165,-1,0,0);
    motor(1,0);motor(2,0);
    samp(); wait(m);  
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(120);samp();} 
    sound(1000,100);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=154;               
}


void zhufeng()
{
    
    int fast=180;
    int slow=70;
    while(!r_xf)  {go_front(fast);samp();}
  //  while(!r_xb)  {go_front(fast);samp();}
        motor(1,0);motor(2,0);wait(0.1); sound(1000,100);
   gd_threshold_f[0]=353;
    gd_threshold_f[1]=294;
    gd_threshold_f[2]=315;
    gd_threshold_f[3]=300;
    gd_threshold_f[4]=336;
    gd_threshold_f[5]=335;
    gd_threshold_f[6]=361;

    gd_threshold_b[0]=362; 
    gd_threshold_b[1]=350;
    gd_threshold_b[2]=342;
    gd_threshold_b[3]=302;
    gd_threshold_b[4]=280;
    gd_threshold_b[5]=254;
    gd_threshold_b[6]=301; 
    
    gd_threshold_l=213;
    tt=seconds(1)+0.3;
    while(seconds(1)<tt){go_front(fast);samp();} sound(1000,100);
    while(!r_xf)  {go_front(fast);samp();}
    motor(1,0);motor(2,0);wait(0.1);  
    tt=seconds(1)+0.3;
    while(seconds(1)<tt){go_front(120);samp();}
     sound(1000,100);
          //    tt=seconds(1)+1.5;
    while(!r_fh)  {go_front(180);samp();}
        tt=seconds(1)+0.2;
    while(seconds(1)<tt){motor(1,80);motor(2,80);samp();}
        motor(1,0);motor(2,0);
wait(0.1);  
//闪灯
comturn_5();
    samp();
    motor(1,0);motor(2,0);wait(0.1); 
    tt=seconds(1)+0.4;
        while(seconds(1)<tt)  {motor(1,slow);motor(2,slow);samp();}
        sound(1000,100);
    samp();
    while(!r_xf)  {go_front(slow);samp();}
    tt=seconds(1)+0.8;
    while(seconds(1)<tt){go_front(slow);samp();} 
    sound(1000,100);
    motor(1,0);motor(2,0);wait(0.1);  
    while(!r_xf)  {go_front(slow);samp();}
     motor(1,0);motor(2,0);wait(0.1);    
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=179;
    
    tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_front(slow);samp();} 
    sound(1000,100);
     motor(1,0);motor(2,0);wait(0.1);
}

void fuqiao()
{
    while(r_db==0){go_front(200);samp();}
    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go2(85,80);samp();}
    sound(1000,100);
    while(r_df==0){go_front_1(80);samp_1();}
     tt=seconds(1)+0.05;
    while(seconds(1)<tt){go2(80,80);samp();}
      tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_front(80);samp();}
    sound(1000,100);
     motor(1,0);motor(2,0);wait(0.3);samp();
}



void waveboard()//============================================================//现场需调试
{
    soft_start_f();
     while(!r_xb) {go_front(140);samp();} //原300
      motor(1,0);motor(2,0);wait(0.2); 
     soft_start_f();
     gd_threshold_f[0]=353;
    gd_threshold_f[1]=294;
    gd_threshold_f[2]=315;
    gd_threshold_f[3]=300;
    gd_threshold_f[4]=336;
    gd_threshold_f[5]=335;
    gd_threshold_f[6]=361;

    gd_threshold_b[0]=362; 
    gd_threshold_b[1]=350;
    gd_threshold_b[2]=342;
    gd_threshold_b[3]=302;
    gd_threshold_b[4]=280;
    gd_threshold_b[5]=254;
    gd_threshold_b[6]=301; 
    
    gd_threshold_l=213;
    tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_front(80);samp();};
    motor(1,0);motor(2,0);wait(0.1);    
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=179;
}

void delay()
{
    tt=seconds(1)+0.1;
    while(tt>seconds(1))  {;}
}




void climbzhufeng()
{
int fast=200;
int slow=80;
float t=0.4;
    tt=seconds(1)+0.3;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(0.2);
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(fast);samp();} 
    sound(1000,100);
    motor(1,0);motor(2,0);wait(0.2); 
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
    while(gd_r<gd_threshold_r)    {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(0.2); 
    platform();
        while(gd_f[0]>gd_threshold_f[0])  {go_front(fast-30);samp();}
    while(gd_l<gd_threshold_l)    {go_front(slow-20);samp();}
    motor(1,0);motor(2,0);wait(0.2); 
    turn_fl(t);wait(0.5); 
   samp();
   qiaobanqiao();
   wait(0.5);

if (door[3]==0) route1(); //door3 open
else if (door[4]==0) route3(); //door3 close,door 4open
else route2();//door3 and door4 close
//===========qiaobanqiao
qiaobanqiao();
samp();
    while(gd_r<gd_threshold_r)  {go2(slow,slow);samp();}
    motor(1,0);motor(2,0);
    wait(0.2);
    turn_fl(t); 
    samp();
    motor(1,0);motor(2,0);
    
          while(!r_xf)  {go_front(fast);samp();}
    motor(1,0);motor(2,0);wait(0.3); sound(1000,100);
    
    gd_threshold_f[0]=374;
    gd_threshold_f[1]=306;
    gd_threshold_f[2]=335;
    gd_threshold_f[3]=302;
    gd_threshold_f[4]=302;
    gd_threshold_f[5]=330;
    gd_threshold_f[6]=336;

    gd_threshold_b[0]=353; 
    gd_threshold_b[1]=345;
    gd_threshold_b[2]=329;
    gd_threshold_b[3]=316;
    gd_threshold_b[4]=325;
    gd_threshold_b[5]=327;
    gd_threshold_b[6]=362; 
      gd_threshold_l=140;
    tt=seconds(1)+0.3;
    while(seconds(1)<tt){go_front(fast);samp();}
     sound(1000,100);
    while(gd_f[0]>gd_threshold_f[0])  {go_front(fast);samp();}
        tt=seconds(1)+0.3;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
        motor(1,0);motor(2,0);
    setport(5,0);wait(0.2);setport(5,1);//闪灯
wait(0.3);  
    motor(1,0);motor(2,0);
    gd_threshold_f[0]=430;
    gd_threshold_f[1]=394;
    gd_threshold_f[2]=416;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=403;
    gd_threshold_f[5]=424;
    gd_threshold_f[6]=426;

    gd_threshold_b[0]=461; 
    gd_threshold_b[1]=443;
    gd_threshold_b[2]=449;
    gd_threshold_b[3]=423;
    gd_threshold_b[4]=428;
    gd_threshold_b[5]=422;
    gd_threshold_b[6]=439; 
  gd_threshold_l=181;   

}

void route11()
{
int fast=260;
int slow=80;
float t=0.4;
float m=0.1;
   samp();
  //  while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
    while(gd_r<gd_threshold_r)    {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    samp();
    turn_fl(t);wait(m); 
    soft_start_f();
    samp();

    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast-100);samp();}
    while(gd_r<gd_threshold_r)    {go2(slow,slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    turn_bl(0.3);wait(m);samp();
    sound(1000,100);
                    tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_back(90);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_back(fast-100);samp();}
    while(gd_l<gd_threshold_l)    {go_back(slow);samp();}
    sound(1000,100);
    samp();
    //过4号门
    passdoor2();
    while(gd_b[6]>gd_threshold_b[6])  {go_back(fast);samp();}
    while(gd_r<gd_threshold_r)    {go_back(slow);samp();}
        while(gd_b[0]>gd_threshold_b[0])  {go_back(fast);samp();}
    while(gd_l<gd_threshold_l)    {go_back(slow);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_back(fast-150);samp();}
    while(gd_l<gd_threshold_l)    {go2(-slow-15,-slow-15);samp();}
    motor(1,0);motor(2,0);wait(m);
    turn_fl(t+0.1);wait(m);samp();
    
    all();
    
       soft_start_f();
       samp();
       while(gd_f[0]>gd_threshold_f[0])  {go_front(fast-50);samp();}
       while(gd_l<gd_threshold_l)  {go2(slow,slow);samp();}
        while(gd_f[0]>gd_threshold_f[0])  {go_front(fast-50);samp();}
       while(gd_l<gd_threshold_l)  {go_front(slow);samp();}
       motor(1,0);motor(2,0);wait(m);  
         //过4号门
         passdoor();samp();
         while(gd_f[0]>gd_threshold_f[0])  {go_front(fast);samp();}
         while(gd_l<gd_threshold_l)  {go_front(fast);samp();}
         while(gd_f[6]>gd_threshold_f[6])  {go_front(fast-50);samp();}
        while(gd_f[0]>gd_threshold_f[0])  {go_front(slow);samp();}
         while(gd_l<gd_threshold_l)  {go2(slow-20,slow-20);samp();}        
         motor(1,0);motor(2,0);
         wait(m);
         turn_fl(t);
         samp();
             tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_front(slow);samp();}
         while(gd_f[6]>gd_threshold_f[6])  {go_front(fast-100);samp();}
         while(gd_r<gd_threshold_r)  {go_front(slow);samp();}
         motor(1,0);motor(2,0);
         wait(0.2);
         turn_fr(t);
         samp();
}

void route12()
{
int slow=80;
int fast=260;
float t=0.4;
float m=0.1;
    while(gd_r<gd_threshold_r)    {go_front(slow);samp();}
        while(gd_b[0]>gd_threshold_b[0])    {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    samp();
//===================waveboard 
          while(!r_xb)    {go_front(fast-50);samp();}
    while(gd_l<190)    {go_front(90);samp();}
                tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_front(90);samp();}
//===================waveboard 
    sound(1000,100);
            while(gd_f[6]>gd_threshold_f[6])  {go_front(fast-60);samp();}
        while(gd_f[0]>gd_threshold_f[0])  {go_front(fast-60);samp();}
         while(gd_l<gd_threshold_l)  {go2(slow-10,slow-10);samp();}
//====================door3
passdoor();
while(gd_f[0]>gd_threshold_f[0])  {go_front(slow+60);samp();}
while(gd_l<gd_threshold_l)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    turn_fl(t);wait(m);samp();
    sound(1000,100);
while(gd_f[0]>gd_threshold_f[0])  {go_front(fast-120);samp();}
while(gd_l<gd_threshold_l)  {go_front(slow-15);samp();}
    motor(1,0);motor(2,0);wait(m);
             turn_fr(t);wait(m);samp();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(fast-100);samp();}
    while(gd_l<gd_threshold_l)    {go_front(slow);samp();}
    while(gd_f[0]>gd_threshold_f[0])  {go_front(fast-100);samp();}
    while(gd_l<gd_threshold_l)    {go2(slow-15,slow-15);samp();}
    motor(1,0);motor(2,0);wait(m);
    turn_fr(0.3);wait(m);  samp();
    all();

      soft_start_f();
       samp();
       
       while(gd_f[0]>gd_threshold_f[0])  {go_front(fast-150);samp();}
       while(gd_l<gd_threshold_l)  {go2(slow-10,slow-10);samp();}
       motor(1,0);motor(2,0);wait(0.2);  
    turn_fl(t);
wait(0.2);   
 samp();
       while(gd_f[6]>gd_threshold_f[6])  {go_front(fast-100);samp();}
       while(gd_r<gd_threshold_r)  {go_front(slow);samp();}
   /*    motor(1,0);motor(2,0);wait(m);  
    turn_fr(t);
wait(m);  
       samp(); 
       //雁荡山
       soft_start_f();
       f_bump(fast,slow);
       motor(1,0);motor(2,0);wait(m);
       while(gd_b[0]>gd_threshold_b[0])  {go_back(slow);samp();}
       while(gd_l<gd_threshold_l)  {go2(-slow,-slow);samp();}
       motor(1,0);motor(2,0);
       wait(m);
       turn_fl(t);
       samp();*/
       wait(m);     
       
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast-150);samp();}
       while(gd_r<gd_threshold_r)  {go_front(slow);samp();}
       motor(1,0);motor(2,0);
       wait(m);
       turn_fr(t);
       samp();
       wait(m);
       while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
       while(gd_r<gd_threshold_r)  {go_front(slow);samp();}
       motor(1,0);motor(2,0);wait(m);
       samp();
         //过3号门
         passdoor();
         while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
         while(gd_r<gd_threshold_r)  {go2(slow,slow);samp();}
         motor(1,0);motor(2,0);
         wait(m);

         soft_start_f();
         samp();
//=============waveboard

          while(!r_xb)    {go_front(fast-50);samp();}
    while(gd_r<135)    {go_front(90);samp();}
                tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_front(90);samp();}
    sound(1000,100);

         while(gd_f[0]>gd_threshold_f[0])  {go_front(fast-100);samp();}
         while(gd_l<gd_threshold_l)  {go_front(slow);samp();}
         motor(1,0);motor(2,0);
         wait(0.2);
         samp();
}
void route13()
{
int fast=260;
int slow=80;
float t=0.4;
float m=0.1;
  samp();
  //  while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
    while(gd_r<gd_threshold_r)    {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    samp();
    turn_fl(t);wait(m); 
    soft_start_f();
  samp();
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast-70);samp();}
    while(gd_r<gd_threshold_r)    {go2(slow-15,slow-15);samp();}
    motor(1,0);motor(2,0);wait(m);
    turn_fr(0.15);wait(m);samp();
  //  ==========5号门
 passdoor();
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast-150);samp();}
    while(gd_r<gd_threshold_r)    {go2(slow-15,slow-15);samp();}
        while(gd_l<gd_threshold_l)    {go2(slow-20,slow-20);samp();}
    motor(1,0);motor(2,0);wait(m);
      turn_fr(0.3);wait(m);samp();
        tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_front(100);samp();}
    
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast-60);samp();}
    while(gd_r<gd_threshold_r)    {go2(slow,slow);samp();}
        while(gd_l<gd_threshold_l)    {go2(slow-10,slow-10);samp();}
    motor(1,0);motor(2,0);wait(m);
    turn_fl(0.2);wait(m);samp();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(fast-100);samp();}
    while(gd_l<gd_threshold_l)    {go2(slow-15,slow-15);samp();}
    motor(1,0);motor(2,0);wait(m);
    turn_fr(0.3);wait(m);  samp();
    all();
      soft_start_f();
       samp();
       while(gd_f[0]>gd_threshold_f[0])  {go_front(fast-100);samp();}
       while(gd_l<gd_threshold_l)  {go2(slow-10,slow-10);samp();}
    
      motor(1,0);motor(2,0);wait(m);
    turn_fr(0.25);wait(m); samp();

       while(gd_f[0]>gd_threshold_f[0])  {go_front(fast-50);samp();}
       while(gd_l<gd_threshold_l)  {go2(slow-20,slow-20);samp();}
       while(gd_b[0]>gd_threshold_b[0])  {go2(slow-20,slow-20);samp();}
       motor(1,0);motor(2,0);
       wait(m);
       turn_fl(0.2);
       samp();
    tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_front(100);samp();}

         //过5号门
        passdoor();
         while(gd_f[0]>gd_threshold_f[0])  {go_front(fast);samp();}
         while(gd_l<gd_threshold_l)  {go_front(slow);samp();}
         while(gd_b[1]>gd_threshold_b[1])  {go2(slow-20,slow-20);samp();}
         motor(1,0);motor(2,0);
         wait(m);
         turn_fl(0.4);
         samp();
         soft_start_f();
     tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_front(slow);samp();}
         while(gd_f[6]>gd_threshold_f[6])  {go_front(fast-100);samp();}
         while(gd_r<gd_threshold_r)  {go_front(slow);samp();}
         motor(1,0);motor(2,0);
         wait(0.2);
         turn_fr(t);
         samp();
}
void climball()
{
int fast=230;
int slow=80;
float t=0.4;
float m=0.2;
    tt=seconds(1)+0.3;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(fast);samp();} 
    sound(1000,100); 
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
    while(gd_r<gd_threshold_r)    {go_front(slow);samp();}


    while(!r_xf)  {go_front(fast-50);samp();}
    motor(1,0);motor(2,0);wait(m); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.3;
    while(seconds(1)<tt){go_front(120);samp();}
     sound(1000,100);
    while(gd_l<gd_threshold_l)  {go_front(120);samp();}
        tt=seconds(1)+0.4;
    while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
        motor(1,0);motor(2,0);
        setport(10,1);wait(0.2);setport(10,0);wait(0.2);setport(10,1);wait(0.2);setport(10,0);//闪灯
wait(m);  
comturn_5();
    motor(1,0);motor(2,0);wait(m);  
    samp();
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(120);samp();} 
    sound(1000,100);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=436; 
    gd_threshold_b[1]=430;
    gd_threshold_b[2]=433;
    gd_threshold_b[3]=416;
    gd_threshold_b[4]=433;
    gd_threshold_b[5]=413;
    gd_threshold_b[6]=424; 
    gd_threshold_l=179; 

    while(gd_f[0]>gd_threshold_f[0])  {go_front(fast-50);samp();}
    while(gd_l<gd_threshold_l)    {go_front(slow-20);samp();}
    motor(1,0);motor(2,0);wait(0.2); 
    turn_fl(t);wait(0.5); 
          soft_start_f();
   samp();
   qiaobanqiao();
   wait(0.5);

if (door[4]==0) route11(); //door4 open
else if (door[3]==0) route12(); //door4 close,door 3open
else route13();//door3 and door4 close

//===========qiaobanqiao
     qiaobanqiao();
    samp();
    while(gd_f[6]>gd_threshold_f[6])  {go_front(slow);samp();}
    while(gd_r<gd_threshold_r)    {go2(slow,slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    turn_fr(t);wait(m); 
        samp();
  
      while(!r_xb)  {go_back(fast);samp();}
    motor(1,0);motor(2,0);wait(m); sound(1000,100);
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=140;

    tt=seconds(1)+0.3;
    while(seconds(1)<tt){go_back(120);samp();}
     sound(1000,100);
    while(gd_l<gd_threshold_l)  {go_back(120);samp();}
        tt=seconds(1)+0.4;
    while(seconds(1)<tt){motor(1,-80);motor(2,-80);samp();}
        motor(1,0);motor(2,0);
wait(m);  
    samp();
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=436; 
    gd_threshold_b[1]=430;
    gd_threshold_b[2]=433;
    gd_threshold_b[3]=416;
    gd_threshold_b[4]=433;
    gd_threshold_b[5]=413;
    gd_threshold_b[6]=424; 
    gd_threshold_l=179;   
}
void testdoor1()
{
 int fast=220;
    int slow=100;
    float m=0.1;
        while(r_df==0)
            {samp();}
   tt=seconds(1)+0.2;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(120);samp();} 
    sound(1000,100);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=179;  
   
    while(gd_f[0]>gd_threshold_f[0])  {go_front(160);samp();}
    wait(0.05);
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);samp(); 
    soft_start_f();
    
  

    while(gd_f[0]>gd_threshold_f[0])  {go_front(170);samp();}
    while(gd_l<gd_threshold_l)  {go2(50,50);samp();}
    motor(1,0); motor(2,0); wait(0.2);
    turn_fl(0.4);wait(0.2);samp();
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(fast);samp();}
    while(gd_l<gd_threshold_l)  {go_front(fast);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(fast);samp();}
    platform();
   
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     turn_bl(0.1); 
    // comturn_hh(165,-1,0,0);
    soft_start_b();
    tt=seconds(1)+0.3;
    while(seconds(1)<tt){go_back(160);samp();}
    b_bump(170,70);
     //comturn_hh(165,-1,0,0);
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(60,60);samp();}
    while(gd_b[0]>gd_threshold_b[0]) {go2(60,60);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
    turn_fl(0.2);wait(m+0.1);samp();//b 
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(fast-20);samp();}
    while(!r_xb) {go_front(200);samp();}
    motor(1,0);motor(2,0);wait(1);
    samp();
    flag3=r_df;
    locate(1,1);
    printf("door3=%d",r_df);
    while(gd_b[6]>gd_threshold_b[6])  {go_back(150);samp();}
    while(gd_r<gd_threshold_r)  {go_back(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);samp();
     soft_start_f();
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);samp();
     soft_start_b();
    b_bump(160,70);soft_start_f();
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);samp();soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);soft_start_f();
    while(!r_xb) {go_front(200);samp();}
    motor(1,0);motor(2,0);wait(1);samp();
    flag2=r_df;
    locate(2,1);
    printf("door2=%d",r_df);soft_start_b();
    while(gd_b[6]>gd_threshold_b[6])  {go_back(200);samp();}
    while(gd_r<gd_threshold_r)  {go_back(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);
    soft_start_b();
    b_bump(160,70);soft_start_f();
    while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
     while(gd_f[0]>gd_threshold_f[0])  {go_front(140);samp();}
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);soft_start_f();
    while(!r_xb) {go_front(200);samp();}
    motor(1,0);motor(2,0);wait(1);
    samp();
    flag1=r_df;
    locate(3,1);
    printf("door1=%d",r_df);
    if((flag1+flag2+flag3)==1)
    {
    flag4=1;
    locate(4,1);
    printf("door4=%d",flag4);
    }
    else
    {
    flag4=0;
    locate(4,1);
    printf("door4=%d",flag4);
    }
   while(gd_b[6]>gd_threshold_b[6])  {go_back(200);samp();}
    while(gd_r<gd_threshold_r)  {go_back(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    platform3();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(fast-20);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_br(0.15);wait(m+0.1);samp();
     //comturn_hh(165,-1,0,0);
     b_bump(160,70);//long wang shan
     //comturn_hh(165,-1,0,0);
     soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(fast-20);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_bl(0.15);wait(m+0.1);samp();//d
     while(gd_f[0]>gd_threshold_f[0])  {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);soft_start_f();
    while(gd_f[6]>gd_threshold_f[6]) {go_front(fast-20);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fr(0.4);wait(m+0.1);samp();
      while(!r_df)  {go_front(fast-20);samp();}
    motor(1,0);motor(2,0);wait(m); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go_front(slow+20);samp();}
     sound(1000,1000);
     while(gd_f[0]>435){go2(150,150);samp();}
     while(gd_r>60){go2(120,120);samp();}
   while(gd_b[0]>440){go2(100,100);samp();}
  //  while(gd_l<gd_threshold_l)  {go_front(120);samp();}
  //      tt=seconds(1)+0.4;
  //  while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
    motor(1,0);motor(2,0);
wait(m);  
comturn_hh(165,-1,0,0);
    motor(1,0);motor(2,0);
}



void spotsplatform1()
{
int fast=220;
    int slow=100;
    float m=0.1;
while(r_df==0)
            {samp();}
   tt=seconds(1)+0.2;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(120);samp();} 
    sound(1000,100);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=179;  
   
    while(gd_f[0]>gd_threshold_f[0])  {go_front(160);samp();}
    wait(0.05);
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);samp(); 
    soft_start_f();
    
  

    while(gd_f[0]>gd_threshold_f[0])  {go_front(170);samp();}
    while(gd_l<gd_threshold_l)  {go2(50,50);samp();}
    motor(1,0); motor(2,0); wait(0.2);
    turn_fl(0.4);wait(0.2);samp();
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(fast);samp();}
    while(gd_l<gd_threshold_l)  {go_front(fast);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(fast);samp();}
    platform();
   
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     turn_bl(0.1); 
    // comturn_hh(165,-1,0,0);
    soft_start_b();
    tt=seconds(1)+0.3;
    while(seconds(1)<tt){go_back(160);samp();}
    b_bump(170,90);
     //comturn_hh(165,-1,0,0);
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(60,60);samp();}
    while(gd_b[0]>gd_threshold_b[0]) {go2(60,60);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
    turn_fl(0.2);wait(m+0.1);samp();//b 
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(fast-20);samp();}
    
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);samp();soft_start_f();

    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);samp();
     soft_start_b();
    b_bump(160,90);soft_start_f();
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);samp();soft_start_f();
    
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);samp();
     soft_start_b();
    b_bump(160,90);soft_start_f();
    
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);
    soft_start_f();
    
   while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);soft_start_f();
    platform3();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(fast-20);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_br(0.15);wait(m+0.1);samp();
     //comturn_hh(165,-1,0,0);
     b_bump(160,80);//long wang shan
     //comturn_hh(165,-1,0,0);
     soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(fast-20);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_bl(0.15);wait(m+0.1);samp();//d
     
     tt=seconds(1)+0.2;
     while(seconds(1)<tt){go_front(200);samp();}
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    passdoor();
     soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
        platform();
          while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
        while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_br(0.4);wait(0.2);soft_start_b();
       b_bump(160,90);soft_start_f();
       
        while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
      while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(200);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
     while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);soft_start_b();
       b_bump(160,90);soft_start_f();
       
       while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
     
     while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
       f_bump(160,90);soft_start_b();
       
        while(gd_b[0]>gd_threshold_b[0])  {go_back(150);samp();}
    while(gd_l<gd_threshold_l)  {go_back(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fr(0.4);wait(0.2);soft_start_f();
      passdoor();
       while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fr(0.4);wait(0.2);soft_start_f();
        while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fr(0.4);wait(0.2);soft_start_f();
       while(!r_df)  {go_front(fast-20);samp();}
    motor(1,0);motor(2,0);wait(m); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go_front(slow+20);samp();}
     sound(1000,1000);
     while(gd_f[0]>435){go2(150,150);samp();}
     while(gd_r>60){go2(120,120);samp();}
   while(gd_b[0]>440){go2(100,100);samp();}
  //  while(gd_l<gd_threshold_l)  {go_front(120);samp();}
  //      tt=seconds(1)+0.4;
  //  while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
    motor(1,0);motor(2,0);
wait(m);  
comturn_hh(165,-1,0,0);
    motor(1,0);motor(2,0);

}
void spotsplatform2()
{
int fast=220;
    int slow=100;
    float m=0.1;
 while(r_df==0)
            {samp();}
   tt=seconds(1)+0.2;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(120);samp();} 
    sound(1000,100);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=179;  
   
    while(gd_f[0]>gd_threshold_f[0])  {go_front(160);samp();}
    wait(0.05);
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);samp(); 
    soft_start_f();
    
  

    while(gd_f[0]>gd_threshold_f[0])  {go_front(170);samp();}
    while(gd_l<gd_threshold_l)  {go2(50,50);samp();}
    motor(1,0); motor(2,0); wait(0.2);
    turn_fl(0.4);wait(0.2);samp();
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(fast);samp();}
    while(gd_l<gd_threshold_l)  {go_front(fast);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(fast);samp();}
    platform();
   
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     turn_bl(0.1); 
    // comturn_hh(165,-1,0,0);
    soft_start_b();
    tt=seconds(1)+0.3;
    while(seconds(1)<tt){go_back(160);samp();}
    b_bump(170,90);
     //comturn_hh(165,-1,0,0);
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(60,60);samp();}
    while(gd_b[0]>gd_threshold_b[0]) {go2(60,60);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
    turn_fl(0.2);wait(m+0.1);samp();//b 
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(fast-20);samp();}
    
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);samp();soft_start_f();

    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);samp();
     soft_start_b();
    b_bump(160,90);soft_start_f();
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);samp();soft_start_f();
    
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);samp();
     soft_start_b();
    b_bump(160,90);soft_start_f();
    
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);
    soft_start_f();
    
   while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);soft_start_f();
    platform3();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(fast-20);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_br(0.15);wait(m+0.1);samp();
     //comturn_hh(165,-1,0,0);
     b_bump(160,80);//long wang shan
     //comturn_hh(165,-1,0,0);
     soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(fast-20);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_bl(0.15);wait(m+0.1);samp();//d
     
     tt=seconds(1)+0.2;
     while(seconds(1)<tt){go_front(200);samp();}
     while(gd_f[0]>gd_threshold_f[0])  {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    waveboard();
     
    passdoor();
     soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
        platform();
          while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
        while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_br(0.4);wait(0.2);soft_start_b();
       b_bump(160,90);soft_start_f();
       
        while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
      while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(200);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
     while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);soft_start_b();
       b_bump(160,90);soft_start_f();
       
       while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
     
     while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
       f_bump(160,90);soft_start_b();
       
        while(gd_b[0]>gd_threshold_b[0])  {go_back(150);samp();}
    while(gd_l<gd_threshold_l)  {go_back(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fr(0.4);wait(0.2);soft_start_f();
      
      while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
    while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);soft_start_f();
    
      passdoor();
        tt=seconds(1)+0.5;
     while(seconds(1)<tt){go_front(140);samp();}
      waveboard();
      tt=seconds(1)+0.2;
      while(seconds(1)<tt){go2(50,50);samp();}
       while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
    
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fr(0.4);wait(0.2);soft_start_f();
       while(!r_df)  {go_front(fast-20);samp();}
    motor(1,0);motor(2,0);wait(m); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go_front(slow+20);samp();}
     sound(1000,1000);
     while(gd_f[0]>435){go2(150,150);samp();}
     while(gd_r>60){go2(120,120);samp();}
   while(gd_b[0]>440){go2(100,100);samp();}
  //  while(gd_l<gd_threshold_l)  {go_front(120);samp();}
  //      tt=seconds(1)+0.4;
  //  while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
    motor(1,0);motor(2,0);
wait(m);  
comturn_hh(165,-1,0,0);
    motor(1,0);motor(2,0);


}
void spotsplatform3()
{
int fast=220;
    int slow=100;
    float m=0.1;
  while(r_df==0)
            {samp();}
   tt=seconds(1)+0.2;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(120);samp();} 
    sound(1000,100);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=179;  
   
    while(gd_f[0]>gd_threshold_f[0])  {go_front(160);samp();}
    wait(0.05);
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);samp(); 
    soft_start_f();
    
  

    while(gd_f[0]>gd_threshold_f[0])  {go_front(170);samp();}
    while(gd_l<gd_threshold_l)  {go2(50,50);samp();}
    motor(1,0); motor(2,0); wait(0.2);
    turn_fl(0.4);wait(0.2);samp();
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(fast);samp();}
    while(gd_l<gd_threshold_l)  {go_front(fast);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(fast);samp();}
    platform();
   
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     turn_bl(0.1); 
    // comturn_hh(165,-1,0,0);
    soft_start_b();
    tt=seconds(1)+0.3;
    while(seconds(1)<tt){go_back(160);samp();}
    b_bump(170,90);
     //comturn_hh(165,-1,0,0);
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(60,60);samp();}
    while(gd_b[0]>gd_threshold_b[0]) {go2(60,60);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
    turn_fl(0.2);wait(m+0.1);samp();//b 
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(fast-20);samp();}
    
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);samp();soft_start_f();

    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);samp();
     soft_start_b();
    b_bump(160,90);soft_start_f();
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);samp();soft_start_f();
    
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);samp();
     soft_start_b();
    b_bump(160,90);soft_start_f();
    
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);
    soft_start_f();
    
   while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);soft_start_f();
    platform3();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(fast-20);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_br(0.15);wait(m+0.1);samp();
     //comturn_hh(165,-1,0,0);
     b_bump(160,80);//long wang shan
     //comturn_hh(165,-1,0,0);
     soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(fast-20);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_bl(0.15);wait(m+0.1);samp();//d
     
     tt=seconds(1)+0.2;
     while(seconds(1)<tt){go_front(200);samp();}
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
      while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    passdoor();
     soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
        platform();
          while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
        while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_br(0.4);wait(0.2);soft_start_b();
       b_bump(160,90);soft_start_f();
       
        while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
      while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(200);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
     while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);soft_start_b();
       b_bump(160,90);soft_start_f();
       
       while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
     
     while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
       f_bump(160,90);soft_start_b();
       
        while(gd_b[0]>gd_threshold_b[0])  {go_back(150);samp();}
    while(gd_l<gd_threshold_l)  {go_back(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fr(0.4);wait(0.2);soft_start_f();
      
      while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
    while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     
    turn_fl(0.4);wait(0.2);soft_start_f();
    
      passdoor();
      
       while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
    
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
      turn_fl(0.4);wait(0.2);soft_start_f();
       while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fr(0.4);wait(0.2);soft_start_f();
      
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fr(0.4);wait(0.2);soft_start_f();
       while(!r_df)  {go_front(fast-20);samp();}
    motor(1,0);motor(2,0);wait(m); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go_front(slow+20);samp();}
     sound(1000,1000);
     while(gd_f[0]>435){go2(150,150);samp();}
     while(gd_r>60){go2(120,120);samp();}
   while(gd_b[0]>440){go2(100,100);samp();}
  //  while(gd_l<gd_threshold_l)  {go_front(120);samp();}
  //      tt=seconds(1)+0.4;
  //  while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
    motor(1,0);motor(2,0);
wait(m);  
comturn_hh(165,-1,0,0);
    motor(1,0);motor(2,0);

}
void spotszhufeng1()
{
 int fast=220;
    int slow=100;
    float m=0.1;
while(r_df==0)
            {samp();}
   tt=seconds(1)+0.2;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(120);samp();} 
    sound(1000,100);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=179;  
    
  
    while(gd_f[0]>gd_threshold_f[0])  {go_front(160);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);samp(); 
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(170);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
    motor(1,0); motor(2,0); wait(0.2);
    turn_fl(0.4);wait(0.2);samp();
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(fast);samp();}
    while(gd_l<gd_threshold_l)  {go_front(fast);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(fast);samp();}
    platform();
   
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     turn_bl(0.1); 
     soft_start_b();
    // comturn_hh(165,-1,0,0);
    tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_back(120);samp();}
    b_bump(170,70);
     //comturn_hh(165,-1,0,0);
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(60,60);samp();}
    while(gd_b[0]>gd_threshold_b[0]) {go2(60,60);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
    turn_fr(0.2);wait(0.2);samp();//b 
    soft_start_f();
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(220);samp();}
    while(gd_f[0]>gd_threshold_f[0]) {go_front(220);samp();}
    while(gd_l<gd_threshold_l)  {go_front(220);samp();}
    while(gd_b[0]>gd_threshold_b[0]) {go_front(220);samp();}
     
  
 while(gd_f[0]>gd_threshold_f[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(m+0.1);
    turn_fl(0.4);wait(0.2);samp();//b 
    soft_start_f();
     passdoor();
     
    soft_start_f();
    while(gd_f[6]>gd_threshold_f[6])  {go_front(180);samp();}
  
    while(gd_r<gd_threshold_r)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4);  wait(m+0.1);samp();
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(210);samp();}
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();soft_start_f();
     tt=seconds(1)+0.1;
     while(seconds(1)<tt){go_front(200);samp();}
      while(gd_f[0]>gd_threshold_f[0])  {go_front(220);samp();}
    while(gd_l<gd_threshold_l)  {go_front(220);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(220);samp();}
     while(gd_f[0]>gd_threshold_f[0])  {go_front(200);samp();}
   
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();//m
      soft_start_f();
     while(gd_f[0]>gd_threshold_f[0])  {go_front(220);samp();}
    
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();//n
      soft_start_f();
     while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();//t
      soft_start_f();
     qiaobanqiao();
    
    zhufeng();
     tt=seconds(1)+0.4;
     while(seconds(1)<tt){go_front(140);samp();}
  
    qiaobanqiao();
    while(gd_f[6]>gd_threshold_f[6]) {go_front(170);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}  
while(gd_r<gd_threshold_r) {go_front(220);samp();}
       
     while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();
     
    if(flag2==0)
    {
    soft_start_f();
    while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
    while(gd_r<gd_threshold_r) {go_front(220);samp();}
     
     while(gd_f[0]>gd_threshold_f[0])  {go_front(210);samp();}
    while(gd_l<gd_threshold_l)  { go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fl(0.4); wait(0.2);samp();soft_start_f();
   
      passdoor();
       tt=seconds(1)+0.5;
     while(seconds(1)<tt){go_front(140);samp();}
  
    // motor(1,0); motor(2,0); wait(0.2);samp();
     waveboard();
  
    //  tt=seconds(1)+0.15;
   //  while(seconds(1)<tt){go_front(220);samp();}
     while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
     while(gd_b[6]>gd_threshold_b[6])  {go_front(220);samp();}
  //   motor(1,0); motor(2,0); wait(0.2);samp();soft_start_f();
     tt=seconds(1)+0.15;
     while(seconds(1)<tt){go_front(220);samp();}
     while(gd_f[6]>gd_threshold_f[6]) {go_front(120);samp();}
      tt=seconds(1)+0.05;
     while(seconds(1)<tt){;}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();
      while(!r_df)  {go_front(200);samp();}
    motor(1,0);motor(2,0);wait(0.2); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go_front(120);samp();}
     sound(1000,1000);
     while(gd_f[0]>435){go2(150,150);samp();}
     while(gd_r>60){go2(120,120);samp();}
   while(gd_b[0]>440){go2(100,100);samp();}
  //  while(gd_l<gd_threshold_l)  {go_front(120);samp();}
  //      tt=seconds(1)+0.4;
  //  while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
    motor(1,0);motor(2,0);
wait(0.2);  
comturn_hh(165,-1,0,0);
    motor(1,0);motor(2,0);
     
     
  

    }
    else{
    soft_start_f();
     while(gd_f[6]>gd_threshold_f[6])  {go_front(220);samp();}
    while(gd_r<gd_threshold_r)  {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fl(0.4); wait(0.2);samp();soft_start_f();
     passdoor();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(220);samp();}
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(220);samp();}
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fl(0.4); wait(0.2);samp();soft_start_f();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
      while(!r_xf)  {go_front(fast-20);samp();}
    motor(1,0);motor(2,0);wait(m); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go_front(slow+20);samp();}
     sound(1000,1000);
     while(gd_f[0]>435){go2(150,150);samp();}
     while(gd_r>60){go2(120,120);samp();}
   while(gd_b[0]>440){go2(100,100);samp();}
  //  while(gd_l<gd_threshold_l)  {go_front(120);samp();}
  //      tt=seconds(1)+0.4;
  //  while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
    motor(1,0);motor(2,0);
wait(m);  
comturn_hh(165,-1,0,0);
    motor(1,0);motor(2,0);
     
    }
}
void spotszhufeng2()
{
 int fast=220;
    int slow=100;
    float m=0.1;
while(r_df==0)
            {samp();}
  tt=seconds(1)+0.2;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(120);samp();} 
    sound(1000,100);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=179;  
    
  
    while(gd_f[0]>gd_threshold_f[0])  {go_front(160);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);samp(); 
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(170);samp();}
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
    turn_fl(0.4);wait(0.2);samp();
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(fast);samp();}
    while(gd_l<gd_threshold_l)  {go_front(fast);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(fast);samp();}
    platform();
   
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     turn_bl(0.1); 
     soft_start_b();
    // comturn_hh(165,-1,0,0);
    tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_back(120);samp();}
    b_bump(170,70);
     //comturn_hh(165,-1,0,0);
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(60,60);samp();}
    while(gd_b[0]>gd_threshold_b[0]) {go2(60,60);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
    turn_fl(0.2);wait(0.2);samp();//b 
    soft_start_f();
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(200);samp();}
    while(gd_f[6]>gd_threshold_f[6]) {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
    turn_fr(0.4);wait(m+0.1);samp();//g
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4);
     soft_start_f();
     passdoor();///xiugai
     soft_start_f();
    while(gd_f[6]>gd_threshold_f[6])  {go_front(170);samp();}
    while(gd_r<gd_threshold_r)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4);  wait(m+0.1);samp();
    soft_start_f();
    while(gd_f[6]>gd_threshold_f[6])  {go_front(220);samp();}
    while(gd_r<gd_threshold_r)  {go_front(220);samp();}
    while(gd_f[0]>gd_threshold_f[0])  {go_front(220);samp();}
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();soft_start_f();
     tt=seconds(1)+0.1;
     while(seconds(1)<tt){go_front(200);samp();}
      while(gd_f[0]>gd_threshold_f[0])  {go_front(220);samp();}
    while(gd_l<gd_threshold_l)  {go_front(220);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(220);samp();}
     while(gd_f[0]>gd_threshold_f[0])  {go_front(200);samp();}
   
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();//m
      soft_start_f();
     while(gd_f[0]>gd_threshold_f[0])  {go_front(220);samp();}
    
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();//n
      soft_start_f();
     while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();//t
      soft_start_f();
     qiaobanqiao();
    
    zhufeng();
     tt=seconds(1)+0.4;
     while(seconds(1)<tt){go_front(140);samp();}
  
    qiaobanqiao();
    
    while(gd_f[6]>gd_threshold_f[6]) {go_front(180);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(180);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
    while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
   while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
    while(gd_r<gd_threshold_r) {go_front(220);samp();}
     while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
      while(gd_f[6]>gd_threshold_f[6])  {go_front(220);samp();}
    while(gd_r<gd_threshold_r)  { go_front(220);samp();}
     while(gd_f[0]>gd_threshold_f[0])  {go_front(210);samp();}
    while(gd_l<gd_threshold_l)  { go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fl(0.4); wait(0.2);samp();soft_start_f();
   
      passdoor();
       tt=seconds(1)+0.5;
     while(seconds(1)<tt){go_front(140);samp();}
  
    // motor(1,0); motor(2,0); wait(0.2);samp();
     waveboard();
  
    //  tt=seconds(1)+0.15;
   //  while(seconds(1)<tt){go_front(220);samp();}
     while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
     while(gd_b[6]>gd_threshold_b[6])  {go_front(220);samp();}
  //   motor(1,0); motor(2,0); wait(0.2);samp();soft_start_f();
     tt=seconds(1)+0.15;
     while(seconds(1)<tt){go_front(220);samp();}
     while(gd_f[6]>gd_threshold_f[6]) {go_front(120);samp();}
      tt=seconds(1)+0.05;
     while(seconds(1)<tt){;}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();
      while(!r_df)  {go_front(200);samp();}
    motor(1,0);motor(2,0);wait(0.2); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go_front(120);samp();}
     sound(1000,1000);
     while(gd_f[0]>435){go2(150,150);samp();}
     while(gd_r>60){go2(120,120);samp();}
   while(gd_b[0]>440){go2(100,100);samp();}
  //  while(gd_l<gd_threshold_l)  {go_front(120);samp();}
  //      tt=seconds(1)+0.4;
  //  while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
    motor(1,0);motor(2,0);
wait(0.2);  
comturn_hh(165,-1,0,0);
    motor(1,0);motor(2,0);

}
void spotszhufeng3()
{
 int fast=220;
    int slow=100;
    float m=0.1;
while(r_df==0)
            {samp();}
  tt=seconds(1)+0.2;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(120);samp();} 
    sound(1000,100);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=179;  
    
  
    while(gd_f[0]>gd_threshold_f[0])  {go_front(160);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);samp(); 
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(170);samp();}
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
    turn_fl(0.4);wait(0.2);samp();
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(fast);samp();}
    while(gd_l<gd_threshold_l)  {go_front(fast);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(fast);samp();}
    platform();
   
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     turn_bl(0.1); 
     soft_start_b();
    // comturn_hh(165,-1,0,0);
    tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_back(120);samp();}
    b_bump(170,70);
     //comturn_hh(165,-1,0,0);
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(60,60);samp();}
    while(gd_b[0]>gd_threshold_b[0]) {go2(60,60);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
    turn_fl(0.2);wait(m+0.1);samp();//b 
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(fast-20);samp();}
   passdoor();
     soft_start_f();
     
    while(gd_f[6]>gd_threshold_f[6])  {go_front(170);samp();}
    while(gd_r<gd_threshold_r)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(0.2);
     turn_fl(0.4);  wait(m+0.1);samp();
    soft_start_f();
    platform();
     while(gd_f[0]>gd_threshold_f[0])  {go_front(220);samp();}
    while(gd_l<gd_threshold_l)  {go_front(220);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(220);samp();}
    while(gd_f[6]>gd_threshold_f[6])  {go_front(220);samp();}
    while(gd_r<gd_threshold_r)  {go_front(220);samp();}
    while(gd_f[0]>gd_threshold_f[0])  {go_front(220);samp();}
    while(gd_l<gd_threshold_l)  {go_front(220);samp();}
     while(gd_f[6]>gd_threshold_f[6])  {go_front(220);samp();}
    while(gd_r<gd_threshold_r)  {go_front(220);samp();}
    while(gd_f[0]>gd_threshold_f[0])  {go_front(220);samp();}
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();soft_start_f();
     tt=seconds(1)+0.1;
     while(seconds(1)<tt){go_front(200);samp();}
      while(gd_f[0]>gd_threshold_f[0])  {go_front(220);samp();}
    while(gd_l<gd_threshold_l)  {go_front(220);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(220);samp();}
     while(gd_f[0]>gd_threshold_f[0])  {go_front(200);samp();}
   
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();//m
      soft_start_f();
     while(gd_f[0]>gd_threshold_f[0])  {go_front(220);samp();}
    
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();//n
      soft_start_f();
     while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    
    while(gd_l<gd_threshold_l)  {go_front(70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();//t
      soft_start_f();
     qiaobanqiao();
    
    zhufeng();
     tt=seconds(1)+0.4;
     while(seconds(1)<tt){go_front(140);samp();}
  
    qiaobanqiao();
    
    while(gd_f[6]>gd_threshold_f[6]) {go_front(180);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(180);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
      while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
     
    while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
    while(gd_r<gd_threshold_r) {go_front(220);samp();}
     while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
      while(gd_f[6]>gd_threshold_f[6])  {go_front(220);samp();}
    while(gd_r<gd_threshold_r)  { go_front(220);samp();}
     while(gd_f[0]>gd_threshold_f[0])  {go_front(210);samp();}
    while(gd_l<gd_threshold_l)  { go_front(210);samp();}
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  { go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fl(0.4); wait(0.2);samp();soft_start_f();
   
      passdoor();soft_start_f();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  { go_front(200);samp();} 
     while(gd_f[0]>gd_threshold_f[0])  {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  { go_front(70);samp();} 
       motor(1,0); motor(2,0); wait(0.2);
     turn_fl(0.4); wait(0.2);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  { go_front(70);samp();} 
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(120);samp();}
      tt=seconds(1)+0.05;
     while(seconds(1)<tt){;}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();
      while(!r_df)  {go_front(200);samp();}
    motor(1,0);motor(2,0);wait(0.2); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go_front(120);samp();}
     sound(1000,1000);
     while(gd_f[0]>435){go2(150,150);samp();}
     while(gd_r>60){go2(120,120);samp();}
   while(gd_b[0]>440){go2(100,100);samp();}
  //  while(gd_l<gd_threshold_l)  {go_front(120);samp();}
  //      tt=seconds(1)+0.4;
  //  while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
    motor(1,0);motor(2,0);
wait(0.2);  
comturn_hh(165,-1,0,0);
    motor(1,0);motor(2,0);
     
}


int main()
{
    int fast=200;
    int slow=100;
    float m=0.1;
    pcs=selector();
    if(pcs==0)
    { 
    testdoor1();
    testdoor1();
}   
    else if(pcs==1)
    { 
testdoor1();
if(flag1==0)
{
spotsplatform1();
}
else if(flag2==0)
{
spotsplatform2();
}
else if(flag3==0)
{
spotsplatform3();
}
motor(1,0);motor(2,0);
    }      
    else if(pcs==2)
    
    { 
    if(flag3==0)
        {while(r_df==0)
            {samp();}
   tt=seconds(1)+0.2;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(120);samp();} 
    sound(1000,100);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=179;  
   
    while(gd_f[0]>gd_threshold_f[0])  {go_front(160);samp();}
    wait(0.05);
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);samp(); 
    soft_start_f();
    
  

    while(gd_f[0]>gd_threshold_f[0])  {go_front(170);samp();}
    while(gd_l<gd_threshold_l)  {go2(50,50);samp();}
    motor(1,0); motor(2,0); wait(0.2);
    turn_fl(0.4);wait(0.2);samp();
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(fast);samp();}
    while(gd_l<gd_threshold_l)  {go_front(fast);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(fast);samp();}
    platform();
   
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     turn_bl(0.1); 
    // comturn_hh(165,-1,0,0);
    soft_start_b();
    tt=seconds(1)+0.3;
    while(seconds(1)<tt){go_back(160);samp();}
    b_bump(170,90);
     //comturn_hh(165,-1,0,0);
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(60,60);samp();}
    while(gd_b[0]>gd_threshold_b[0]) {go2(60,60);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
    turn_fl(0.2);wait(m+0.1);samp();//b 
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(fast-20);samp();}
    
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);samp();soft_start_f();

    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);samp();
     soft_start_b();
    b_bump(160,90);soft_start_f();
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);samp();soft_start_f();
    
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);samp();
     soft_start_b();
    b_bump(160,90);soft_start_f();
    
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);
    soft_start_f();
    
   while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);soft_start_f();
    platform3();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(fast-20);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_br(0.15);wait(m+0.1);samp();
     //comturn_hh(165,-1,0,0);
     b_bump(160,80);//long wang shan
     //comturn_hh(165,-1,0,0);
     soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(fast-20);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_bl(0.15);wait(m+0.1);samp();//d
     
     tt=seconds(1)+0.2;
     while(seconds(1)<tt){go_front(200);samp();}
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    passdoor();
     soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
        platform();
          while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
        while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_br(0.4);wait(0.2);soft_start_b();
       b_bump(160,90);soft_start_f();
       
        while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
      while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(200);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
     while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);soft_start_b();
       b_bump(160,90);soft_start_f();
       
       while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
     
     while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
       f_bump(160,90);soft_start_b();
       
        while(gd_b[0]>gd_threshold_b[0])  {go_back(150);samp();}
    while(gd_l<gd_threshold_l)  {go_back(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fr(0.4);wait(0.2);soft_start_f();
      passdoor();
       while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fr(0.4);wait(0.2);soft_start_f();
        while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fr(0.4);wait(0.2);soft_start_f();
       while(!r_df)  {go_front(fast-20);samp();}
    motor(1,0);motor(2,0);wait(m); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go_front(slow+20);samp();}
     sound(1000,1000);
     while(gd_f[0]>435){go2(150,150);samp();}
     while(gd_r>60){go2(120,120);samp();}
   while(gd_b[0]>440){go2(100,100);samp();}
  //  while(gd_l<gd_threshold_l)  {go_front(120);samp();}
  //      tt=seconds(1)+0.4;
  //  while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
    motor(1,0);motor(2,0);
wait(m);  
comturn_hh(165,-1,0,0);
    motor(1,0);motor(2,0);
    }
    }
    else if(pcs==3)   
    { 
        
       while(r_df==0)
            {samp();}
   tt=seconds(1)+0.2;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(120);samp();} 
    sound(1000,100);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=179;  
   
    while(gd_f[0]>gd_threshold_f[0])  {go_front(160);samp();}
    wait(0.05);
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);samp(); 
    soft_start_f();
    
  

    while(gd_f[0]>gd_threshold_f[0])  {go_front(170);samp();}
    while(gd_l<gd_threshold_l)  {go2(50,50);samp();}
    motor(1,0); motor(2,0); wait(0.2);
    turn_fl(0.4);wait(0.2);samp();
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(fast);samp();}
    while(gd_l<gd_threshold_l)  {go_front(fast);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(fast);samp();}
    platform();
   
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     turn_bl(0.1); 
    // comturn_hh(165,-1,0,0);
    soft_start_b();
    tt=seconds(1)+0.3;
    while(seconds(1)<tt){go_back(160);samp();}
    b_bump(170,90);
     //comturn_hh(165,-1,0,0);
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(60,60);samp();}
    while(gd_b[0]>gd_threshold_b[0]) {go2(60,60);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
    turn_fl(0.2);wait(m+0.1);samp();//b 
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(fast-20);samp();}
    
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);samp();soft_start_f();

    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);samp();
     soft_start_b();
    b_bump(160,90);soft_start_f();
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);samp();soft_start_f();
    
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);samp();
     soft_start_b();
    b_bump(160,90);soft_start_f();
    
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);
    soft_start_f();
    
   while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);soft_start_f();
    platform3();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(fast-20);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_br(0.15);wait(m+0.1);samp();
     //comturn_hh(165,-1,0,0);
     b_bump(160,80);//long wang shan
     //comturn_hh(165,-1,0,0);
     soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(fast-20);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_bl(0.15);wait(m+0.1);samp();//d
     
     tt=seconds(1)+0.2;
     while(seconds(1)<tt){go_front(200);samp();}
     while(gd_f[0]>gd_threshold_f[0])  {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    waveboard();
    
    passdoor();
     soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
        platform();
          while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
        while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_br(0.4);wait(0.2);soft_start_b();
       b_bump(160,90);soft_start_f();
       
        while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
      while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(200);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
     while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);soft_start_b();
       b_bump(160,90);soft_start_f();
       
       while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
     
     while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
       f_bump(160,90);soft_start_b();
       
        while(gd_b[0]>gd_threshold_b[0])  {go_back(150);samp();}
    while(gd_l<gd_threshold_l)  {go_back(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fr(0.4);wait(0.2);soft_start_f();
      
      while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
    while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);soft_start_f();
    
      passdoor();
      waveboard();
      tt=seconds(1)+0.2;
      while(seconds(1)<tt){go2(50,50);samp();}
       while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
    
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fr(0.4);wait(0.2);soft_start_f();
       while(!r_df)  {go_front(fast-20);samp();}
    motor(1,0);motor(2,0);wait(m); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go_front(slow+20);samp();}
     sound(1000,1000);
     while(gd_f[0]>435){go2(150,150);samp();}
     while(gd_r>60){go2(120,120);samp();}
   while(gd_b[0]>440){go2(100,100);samp();}
  //  while(gd_l<gd_threshold_l)  {go_front(120);samp();}
  //      tt=seconds(1)+0.4;
  //  while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
    motor(1,0);motor(2,0);
wait(m);  
comturn_hh(165,-1,0,0);
    motor(1,0);motor(2,0);

       
    }
     else if(pcs==4)
    {
      while(r_df==0)
            {samp();}
   tt=seconds(1)+0.2;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(120);samp();} 
    sound(1000,100);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=179;  
   
    while(gd_f[0]>gd_threshold_f[0])  {go_front(160);samp();}
    wait(0.05);
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);samp(); 
    soft_start_f();
    
  

    while(gd_f[0]>gd_threshold_f[0])  {go_front(170);samp();}
    while(gd_l<gd_threshold_l)  {go2(50,50);samp();}
    motor(1,0); motor(2,0); wait(0.2);
    turn_fl(0.4);wait(0.2);samp();
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(fast);samp();}
    while(gd_l<gd_threshold_l)  {go_front(fast);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(fast);samp();}
    platform();
   
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     turn_bl(0.1); 
    // comturn_hh(165,-1,0,0);
    soft_start_b();
    tt=seconds(1)+0.3;
    while(seconds(1)<tt){go_back(160);samp();}
    b_bump(170,90);
     //comturn_hh(165,-1,0,0);
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(60,60);samp();}
    while(gd_b[0]>gd_threshold_b[0]) {go2(60,60);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
    turn_fl(0.2);wait(m+0.1);samp();//b 
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(fast-20);samp();}
    
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);samp();soft_start_f();

    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);samp();
     soft_start_b();
    b_bump(160,90);soft_start_f();
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);samp();soft_start_f();
    
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);samp();
     soft_start_b();
    b_bump(160,90);soft_start_f();
    
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
    while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);
    soft_start_f();
    
   while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);soft_start_f();
    platform3();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(fast-20);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_br(0.15);wait(m+0.1);samp();
     //comturn_hh(165,-1,0,0);
     b_bump(160,80);//long wang shan
     //comturn_hh(165,-1,0,0);
     soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(fast-20);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_bl(0.15);wait(m+0.1);samp();//d
     
     tt=seconds(1)+0.2;
     while(seconds(1)<tt){go_front(200);samp();}
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
      while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    passdoor();
     soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
        platform();
          while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
        while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_br(0.4);wait(0.2);soft_start_b();
       b_bump(160,90);soft_start_f();
       
        while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
      while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go_front(200);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
     while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_bl(0.4);wait(0.2);soft_start_b();
       b_bump(160,90);soft_start_f();
       
       while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
     while(gd_f[6]>gd_threshold_f[6])  {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
     
     while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fl(0.4);wait(0.2);soft_start_f();
       f_bump(160,90);soft_start_b();
       
        while(gd_b[0]>gd_threshold_b[0])  {go_back(150);samp();}
    while(gd_l<gd_threshold_l)  {go_back(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fr(0.4);wait(0.2);soft_start_f();
      
      while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fr(0.4);wait(0.2);soft_start_f();
    
    while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     while(gd_f[6]>gd_threshold_f[6])  {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     
    turn_fl(0.4);wait(0.2);soft_start_f();
    
      passdoor();
      
       while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
    
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); soft_start_f();
      turn_fl(0.4);wait(0.2);soft_start_f();
       while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fr(0.4);wait(0.2);soft_start_f();
      
      while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
      turn_fr(0.4);wait(0.2);soft_start_f();
       while(!r_df)  {go_front(fast-20);samp();}
    motor(1,0);motor(2,0);wait(m); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go_front(slow+20);samp();}
     sound(1000,1000);
     while(gd_f[0]>435){go2(150,150);samp();}
     while(gd_r>60){go2(120,120);samp();}
   while(gd_b[0]>440){go2(100,100);samp();}
  //  while(gd_l<gd_threshold_l)  {go_front(120);samp();}
  //      tt=seconds(1)+0.4;
  //  while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
    motor(1,0);motor(2,0);
wait(m);  
comturn_hh(165,-1,0,0);
    motor(1,0);motor(2,0);
    }
     else if(pcs==5)
    { 
    while(r_db==0){go_front(200);samp();}
    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go2(85,80);samp();}
    sound(1000,100);
    while(r_df==0){go_front_1(80);samp_1();}
     tt=seconds(1)+0.05;
    while(seconds(1)<tt){go2(80,80);samp();}
      tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_front(80);samp();}
    sound(1000,100);
     motor(1,0);motor(2,0);wait(0.3);samp();
     platform();
     while(r_db==0){go_front(200);samp();}
    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go2(85,80);samp();}
    sound(1000,100);
    while(r_df==0){go_front_1(80);samp_1();}
     tt=seconds(1)+0.05;
    while(seconds(1)<tt){go2(80,80);samp();}
      tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_front(80);samp();}
    sound(1000,100);
     motor(1,0);motor(2,0);wait(0.3);samp();
    }
     else if(pcs==6)
    {   
   testdoor1();
if(flag1==0)
{
spotszhufeng1();
}
else if(flag2==0)
{
spotszhufeng2();
}
else if(flag3==0)
{
spotszhufeng3();
}
motor(1,0);motor(2,0);
    }
    else if(pcs==7)
    {
     int fast=220;
    int slow=100;
    float m=0.1;
    flag1=1;
flag2=0;
if((flag2==0)&&(flag1==1))
{ r_df=1;
        while(r_df==1)
            {
            samp();
            }
    samp();
   tt=seconds(1)+0.2;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(120);samp();} 
    sound(1000,100);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=179;  
    
  
    while(gd_f[0]>gd_threshold_f[0])  {go_front(160);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);samp(); 
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(170);samp();}
    while(gd_l<gd_threshold_l)  {go2(50,50);samp();}
    motor(1,0); motor(2,0); wait(0.2);
    turn_fl(0.4);wait(0.2);samp();
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(fast);samp();}
    while(gd_l<gd_threshold_l)  {go_front(fast);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(fast);samp();}
    platform();
   
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     turn_bl(0.1); 
     soft_start_b();
    // comturn_hh(165,-1,0,0);
    tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_back(120);samp();}
    b_bump(170,70);
     //comturn_hh(165,-1,0,0);
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(60,60);samp();}
    while(gd_b[0]>gd_threshold_b[0]) {go2(60,60);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
    turn_fl(0.2);wait(0.2);samp();//b 
    soft_start_f();
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(200);samp();}
    while(gd_f[6]>gd_threshold_f[6]) {go_front(170);samp();}
    while(gd_r<gd_threshold_r)  {go2(70,70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
    turn_fr(0.4);wait(m+0.1);samp();//g
    soft_start_f();
    while(gd_f[6]>gd_threshold_f[6]) {go_front(170);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_bl(0.4);
     soft_start_b();
      //comturn_hh(165,-1,0,0);
     b_bump(170,80); //nanhu
      //comturn_hh(165,-1,0,0);
     soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(170);samp();}
    while(gd_r<gd_threshold_r)  {go2(70,70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fr(0.4);wait(m+0.1);samp();
     while(gd_l<gd_threshold_l) {go_front(fast);samp();}
     wait(0.05);
      while(gd_b[0]>gd_threshold_b[0]) {go_front(fast);samp();}
    while(gd_f[6]>gd_threshold_f[6]) {go_front(150);samp();}
    wait(0.05);
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_bl(0.4); wait(m+0.1);samp();
     soft_start_b();
      //comturn_hh(165,-1,0,0);
    b_bump(160,80);//nanhu
     //comturn_hh(165,-1,0,0);
    soft_start_f();
    while(gd_f[6]>gd_threshold_f[6]) {go_front(150);samp();}
    wait(0.05);
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4);wait(0.2);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(150);samp();}
     wait(0.05);
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4);wait(0.2);samp();//e
     soft_start_f();
     while(gd_f[0]>gd_threshold_f[0]) {go_front(170);samp();}
     wait(0.05);
    
    while(gd_l<gd_threshold_l)  {go2(70,70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();//c
     soft_start_f();
     platform2();
while(gd_f[0]>gd_threshold_f[0]) {go_front(fast-20);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_br(0.15);wait(m+0.1);samp();
     //comturn_hh(165,-1,0,0);
     b_bump(170,60);//long wang shan
     //comturn_hh(165,-1,0,0);
     soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(fast-20);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_bl(0.15);wait(m+0.1);samp();//d
     soft_start_f();
     tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(200);samp();}
   // soft_start_f();
    while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}//man
    while(gd_r<gd_threshold_r)  {go2(220,220);samp();}
     while(gd_b[6]>gd_threshold_b[6]) {go_front(220);samp();}
     motor(1,0); motor(2,0); wait(m+0.1); soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(200);samp();}
    while(gd_r<gd_threshold_r)  {go2(70,70);samp();}
     turn_fr(0.4);  wait(m+0.1);samp();soft_start_f();//waveboard
     passdoor();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(150);samp();}
    while(gd_r<gd_threshold_r)  {go2(70,70);samp();}
     turn_fr(0.4);  wait(m+0.1);samp();soft_start_f();

    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    wait(0.05);
    while(gd_l<gd_threshold_l)  {go2(70,70);samp();}
     motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4);  wait(m+0.1);samp();//xishiguli
     f_bump(160,80);
     soft_start_b();
      while(gd_b[6]>gd_threshold_b[6])  {go_back(170);samp();}
      wait(0.05);
    while(gd_r<gd_threshold_r)  {go2(-70,-70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fr(0.4);  wait(m+0.1);samp(); soft_start_f();
     while(gd_f[0]>gd_threshold_f[0])  {go_front(170);samp();}
     wait(0.05);
    while(gd_l<gd_threshold_l)  {go2(slow-40,slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp(); //k
    while(gd_f[0]>gd_threshold_f[0])  {go_front(175);samp();}
    delay(0.05);
    while(gd_l<gd_threshold_l)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp(); //l
      soft_start_f();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(170);samp();}
      wait(0.05);
    while(gd_l<gd_threshold_l)  {go2(60,60);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();
     f_bump(160,80);
     soft_start_b();//qiandaohu
     while(gd_b[6]>gd_threshold_b[6])  {go_back(170);samp();}
     wait(0.05);
    while(gd_r<gd_threshold_r)  {go2(-70,-70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fr(0.4);  wait(m+0.1);samp();
      soft_start_f();
     while(gd_f[0]>gd_threshold_f[0])  {go_front(170);samp();}
     wait(0.05);
    while(gd_l<gd_threshold_l)  {go2(60,60);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();//m
      soft_start_f();
     while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
     delay(0.05);
    while(gd_l<gd_threshold_l)  {go2(60,60);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();//n
      soft_start_f();
     while(gd_f[0]>gd_threshold_f[0])  {go_front(140);samp();}
     wait(0.05);
    while(gd_l<gd_threshold_l)  {go2(60,60);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();//t
      soft_start_f();
     qiaobanqiao();
    sound(1000,100);
    zhufeng();
    qiaobanqiao();
    while(gd_f[6]>gd_threshold_f[6]) {go_front(160);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(200);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}  
while(gd_r<gd_threshold_r) {go_front(220);samp();}
       
     while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();
     
    if(flag2==0)
    {soft_start_f();
    while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
    while(gd_r<gd_threshold_r) {go_front(220);samp();}
     sound(1000,100);
     while(gd_f[0]>gd_threshold_f[0])  {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(70,70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fl(0.4); wait(0.2);samp();soft_start_f();
     
      passdoor();
       tt=seconds(1)+0.5;
     while(seconds(1)<tt){go_front(140);samp();}
    waveboard();
     
     while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
     while(gd_b[6]>gd_threshold_b[6])  {go_front(220);samp();}
     motor(1,0); motor(2,0); wait(0.2);samp();soft_start_f();
     tt=seconds(1)+0.15;
     while(seconds(1)<tt){go_front(220);samp();}
     while(gd_f[6]>gd_threshold_f[6]) {go_front(120);samp();}
      tt=seconds(1)+0.05;
     while(seconds(1)<tt){;}
    while(gd_r<gd_threshold_r) {go2(70,70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();
      while(!r_df)  {go_front(200);samp();}
    motor(1,0);motor(2,0);wait(0.2); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go_front(120);samp();}
     sound(1000,1000);
     while(gd_f[0]>435){go2(150,150);samp();}
     while(gd_r>60){go2(120,120);samp();}
   while(gd_b[0]>440){go2(100,100);samp();}
  //  while(gd_l<gd_threshold_l)  {go_front(120);samp();}
  //      tt=seconds(1)+0.4;
  //  while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
    motor(1,0);motor(2,0);
wait(0.2);  
comturn_hh(165,-1,0,0);
    motor(1,0);motor(2,0);
     
     
   

    }
    else{
    soft_start_f();
     while(gd_f[6]>gd_threshold_f[6])  {go_front(220);samp();}
    while(gd_r<gd_threshold_r)  {go2(70,70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fl(0.4); wait(0.2);samp();soft_start_f();
     passdoor();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(220);samp();}
    while(gd_l<gd_threshold_l)  {go2(70,70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(220);samp();}
    while(gd_l<gd_threshold_l)  {go2(70,70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fl(0.4); wait(0.2);samp();soft_start_f();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(220);samp();}
    while(gd_l<gd_threshold_l)  {go2(70,70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
      while(!r_xf)  {go_front(fast-20);samp();}
    motor(1,0);motor(2,0);wait(m); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go_front(slow+20);samp();}
     sound(1000,1000);
     while(gd_f[0]>435){go2(150,150);samp();}
     while(gd_r>60){go2(120,120);samp();}
   while(gd_b[0]>440){go2(100,100);samp();}
  //  while(gd_l<gd_threshold_l)  {go_front(120);samp();}
  //      tt=seconds(1)+0.4;
  //  while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
    motor(1,0);motor(2,0);
wait(m);  
comturn_hh(160,-1,0,0);
    motor(1,0);motor(2,0);
     
    }
    }
    }
   else if(pcs==8)
    {
         int fast=220;
    int slow=100;
    float m=0.1;
    flag1=1;
flag2=1;
if((flag2==1)&&(flag1==1))
{ r_df=1;
        while(r_df==1)
            {
            samp();
            }
    samp();
   tt=seconds(1)+0.2;
    while(seconds(1)<tt){motor(1,slow);motor(2,slow);samp();}
    sound(1000,100);
    while(!r_xf)  {go_front(slow);samp();}
    motor(1,0);motor(2,0);wait(m);
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(120);samp();} 
    sound(1000,100);
    gd_threshold_f[0]=429;
    gd_threshold_f[1]=391;
    gd_threshold_f[2]=398;
    gd_threshold_f[3]=386;
    gd_threshold_f[4]=401;
    gd_threshold_f[5]=427;
    gd_threshold_f[6]=422;

    gd_threshold_b[0]=442; 
    gd_threshold_b[1]=426;
    gd_threshold_b[2]=369;
    gd_threshold_b[3]=334;
    gd_threshold_b[4]=288;
    gd_threshold_b[5]=267;
    gd_threshold_b[6]=315; 
    gd_threshold_l=179;  
    
  
    while(gd_f[0]>gd_threshold_f[0])  {go_front(160);samp();}
    while(gd_l<gd_threshold_l)  {go_front(50);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
    turn_fl(0.4);wait(0.2);samp(); 
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(170);samp();}
    while(gd_l<gd_threshold_l)  {go2(50,50);samp();}
    motor(1,0); motor(2,0); wait(0.2);
    turn_fl(0.4);wait(0.2);samp();
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(fast);samp();}
    while(gd_l<gd_threshold_l)  {go_front(fast);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(fast);samp();}
    platform();
   
    while(gd_f[6]>gd_threshold_f[6])  {go_front(fast);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow);samp();}
     motor(1,0); motor(2,0); wait(0.2); 
     turn_bl(0.1); 
     soft_start_b();
    // comturn_hh(165,-1,0,0);
    tt=seconds(1)+0.5;
    while(seconds(1)<tt){go_back(120);samp();}
    b_bump(170,90);
     //comturn_hh(165,-1,0,0);
    soft_start_f();
    while(gd_f[0]>gd_threshold_f[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(60,60);samp();}
    while(gd_b[0]>gd_threshold_b[0]) {go2(60,60);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
    turn_fl(0.2);wait(0.2);samp();//b 
    soft_start_f();
    tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(200);samp();}
    while(gd_f[6]>gd_threshold_f[6]) {go_front(170);samp();}
    while(gd_r<gd_threshold_r)  {go2(70,70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
    turn_fr(0.4);wait(m+0.1);samp();//g
    soft_start_f();
    while(gd_f[6]>gd_threshold_f[6]) {go_front(170);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_bl(0.4);
     soft_start_b();
      //comturn_hh(165,-1,0,0);
     b_bump(170,80); //nanhu
      //comturn_hh(165,-1,0,0);
     soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(170);samp();}
    while(gd_r<gd_threshold_r)  {go2(70,70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fr(0.4);wait(m+0.1);samp();
     while(gd_l<gd_threshold_l) {go_front(fast);samp();}
     wait(0.05);
      while(gd_b[0]>gd_threshold_b[0]) {go_front(fast);samp();}
    while(gd_f[6]>gd_threshold_f[6]) {go_front(150);samp();}
    wait(0.05);
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_bl(0.4); wait(m+0.1);samp();
     soft_start_b();
      //comturn_hh(165,-1,0,0);
    b_bump(160,80);//nanhu
     //comturn_hh(165,-1,0,0);
    soft_start_f();
    while(gd_f[6]>gd_threshold_f[6]) {go_front(150);samp();}
    wait(0.05);
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4);wait(0.2);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(150);samp();}
     wait(0.05);
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4);wait(0.2);samp();//e
     soft_start_f();
     while(gd_f[0]>gd_threshold_f[0]) {go_front(170);samp();}
     wait(0.05);
    
    while(gd_l<gd_threshold_l)  {go2(70,70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();//c
     soft_start_f();
     platform2();
while(gd_f[0]>gd_threshold_f[0]) {go_front(fast-20);samp();}
    while(gd_b[0]>gd_threshold_b[0])  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_br(0.15);wait(m+0.1);samp();
     //comturn_hh(165,-1,0,0);
     b_bump(170,80);//long wang shan
     //comturn_hh(165,-1,0,0);
     soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(fast-20);samp();}
    while(gd_r<gd_threshold_r)  {go_front(slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_bl(0.15);wait(m+0.1);samp();//d
     soft_start_f();
     tt=seconds(1)+0.2;
    while(seconds(1)<tt){go_front(200);samp();}
   // soft_start_f();
    while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}//man
    while(gd_r<gd_threshold_r)  {go2(220,220);samp();}
     motor(1,0); motor(2,0); wait(m+0.1); 
    turn_fr(0.4);wait(m+0.1); soft_start_f();
     while(gd_f[0]>gd_threshold_f[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(70,70);samp();}
     turn_fl(0.4);  wait(m+0.1);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}//man
    while(gd_r<gd_threshold_r)  {go2(220,220);samp();}
    motor(1,0); motor(2,0); wait(m+0.1); soft_start_f();
    while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
    while(gd_r<gd_threshold_r)  {go2(220,220);samp();}
     motor(1,0); motor(2,0); wait(m+0.1); 
     turn_fr(0.4);  wait(m+0.1);soft_start_f();
      passdoor();
       while(gd_f[0]>gd_threshold_f[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(70,70);samp();}
     turn_fl(0.4);  wait(m+0.1);samp();soft_start_f();
     platform();
     
      while(gd_f[0]>gd_threshold_f[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(70,70);samp();}
     turn_bl(0.3);  wait(m+0.1);samp();soft_start_b();
     while(gd_b[0]>gd_threshold_b[0]) {go_front(200);samp();}
    while(gd_l<gd_threshold_l)  {go2(70,70);samp();}
     turn_fl(0.3);  wait(m+0.1);samp();soft_start_f();///123456
     while(gd_f[6]>gd_threshold_f[6]) {go_front(170);samp();}
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_bl(0.4);
     soft_start_b();
      //comturn_hh(165,-1,0,0);
     b_bump(170,70); //nanhu
      //comturn_hh(165,-1,0,0);
     soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(170);samp();}
    while(gd_r<gd_threshold_r)  {go2(70,70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fr(0.4);wait(m+0.1);samp();
     while(gd_r<gd_threshold_r) {go_front(fast);samp();}
     wait(0.05);
     
    while(gd_f[6]>gd_threshold_f[6]) {go_front(150);samp();}
    wait(0.05);
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_bl(0.4); wait(m+0.1);samp();
     soft_start_b();
      //comturn_hh(165,-1,0,0);
    b_bump(160,80);//nanhu
     //comturn_hh(165,-1,0,0);
    soft_start_f();
    while(gd_f[6]>gd_threshold_f[6]) {go_front(150);samp();}
    wait(0.05);
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4);wait(0.2);samp();soft_start_f();
       while(gd_f[6]>gd_threshold_f[6]) {go_front(180);samp();}

    while(gd_r<gd_threshold_r)  {go2(180,180);samp();}
    while(gd_f[6]>gd_threshold_f[6]) {go_front(180);samp();}
    wait(0.05);
    while(gd_r<gd_threshold_r)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4);wait(0.2);samp();soft_start_f();
     while(gd_f[0]>gd_threshold_f[0]) {go_front(170);samp();}
    wait(0.05);
    while(gd_l<gd_threshold_l)  {go2(50,50);samp();}
     motor(1,0); motor(2,0); wait(0.2);
     turn_fl(0.4);wait(0.2);samp();soft_start_f();
      f_bump(160,80); soft_start_b();
       
      while(gd_b[6]>gd_threshold_b[6])  {go_back(170);samp();}
      wait(0.05);
    while(gd_r<gd_threshold_r)  {go2(-70,-70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fr(0.4);  wait(m+0.1);samp(); soft_start_f();
     while(gd_f[0]>gd_threshold_f[0])  {go_front(170);samp();}
     wait(0.05);
    while(gd_l<gd_threshold_l)  {go2(slow-40,slow-40);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp(); //k
    while(gd_f[0]>gd_threshold_f[0])  {go_front(200);samp();}
    wait(0.05);
    while(gd_l<gd_threshold_l)  {go2(70,70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp(); //l
      soft_start_f();
      while(gd_f[0]>gd_threshold_f[0])  {go_front(170);samp();}
      wait(0.05);
    while(gd_l<gd_threshold_l)  {go2(70,70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();
     f_bump(160,80);
     soft_start_b();//qiandaohu
     while(gd_b[6]>gd_threshold_b[6])  {go_back(170);samp();}
     wait(0.05);
    while(gd_r<gd_threshold_r)  {go2(-70,-70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fr(0.4);  wait(m+0.1);samp();
      soft_start_f();
     while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
     wait(0.05);
    while(gd_l<gd_threshold_l)  {go2(70,70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();//m
      soft_start_f();
     while(gd_f[0]>gd_threshold_f[0])  {go_front(200);samp();}
     wait(0.05);
    while(gd_l<gd_threshold_l)  {go2(70,70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();//n
      soft_start_f();
     while(gd_f[0]>gd_threshold_f[0])  {go_front(160);samp();}
     wait(0.05);
    while(gd_l<gd_threshold_l)  {go2(70,70);samp();}
     motor(1,0); motor(2,0); wait(m+0.1);
     turn_fl(0.4); wait(m+0.1);samp();//t
      soft_start_f();
     qiaobanqiao();
      tt=seconds(1)+0.2;
    while(seconds(1)<tt){go2(slow,slow);samp();}
    sound(1000,1000);
    zhufeng();
    qiaobanqiao();
    while(gd_f[6]>gd_threshold_f[6]) {go_front(160);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(200);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}  
while(gd_r<gd_threshold_r) {go_front(220);samp();}
       
     while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
    while(gd_r<gd_threshold_r) {go_front(70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();
     
    
    soft_start_f();
    while(gd_f[6]>gd_threshold_f[6]) {go_front(220);samp();}
    while(gd_r<gd_threshold_r) {go_front(220);samp();}
     sound(1000,100);motor(1,0); motor(2,0); wait(0.2);samp();soft_start_f();
     while(gd_f[0]>gd_threshold_f[0])  {go_front(220);samp();}
    while(gd_l<gd_threshold_l)  {go_front(220);samp();}
     motor(1,0); motor(2,0); wait(0.2);samp();soft_start_f();
    while(gd_f[0]>gd_threshold_f[0])  {go_front(180);samp();}
    while(gd_l<gd_threshold_l)  {go2(70,70);samp();}
motor(1,0); motor(2,0); wait(0.2);
     turn_fl(0.4); wait(0.2);samp();soft_start_f();
     
      passdoor();
     while(gd_f[0]>gd_threshold_f[0]) {go_front(220);samp();}
     while(gd_l<gd_threshold_l) {go_front(220);samp();}
      motor(1,0); motor(2,0); wait(0.2);samp();soft_start_f();
    // while(gd_b[0]>gd_threshold_b[0])  {go_front(220);samp();}
      tt=seconds(1)+0.15;
     while(seconds(1)<tt){go_front(220);samp();}
     while(gd_f[0]>gd_threshold_f[0]) {go_front(180);samp();}
     while(gd_l<gd_threshold_l) {go_front(70);samp();}
      motor(1,0); motor(2,0); wait(0.2);
    turn_fl(0.4); wait(0.2);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(200);samp();}
     while(gd_r<gd_threshold_r) {go_front(70);samp();}
      motor(1,0); motor(2,0); wait(0.2);
    turn_fr(0.4); wait(0.2);samp();soft_start_f();
     while(gd_f[6]>gd_threshold_f[6]) {go_front(120);samp();}
      tt=seconds(1)+0.05;
     while(seconds(1)<tt){;}
    while(gd_r<gd_threshold_r) {go2(70,70);samp();}
    motor(1,0); motor(2,0); wait(0.2);
     turn_fr(0.4); wait(0.2);samp();
      while(!r_df)  {go_front(200);samp();}
    motor(1,0);motor(2,0);wait(0.2); sound(1000,100);
  //  while(!r_xb)  {go_front(fast);samp();}
    
    gd_threshold_f[0]=360;
    gd_threshold_f[1]=295;
    gd_threshold_f[2]=320;
    gd_threshold_f[3]=294;
    gd_threshold_f[4]=307;
    gd_threshold_f[5]=333;
    gd_threshold_f[6]=341;

    gd_threshold_b[0]=377; 
    gd_threshold_b[1]=355;
    gd_threshold_b[2]=398;
    gd_threshold_b[3]=346;
    gd_threshold_b[4]=371;
    gd_threshold_b[5]=335;
    gd_threshold_b[6]=374; 
      gd_threshold_l=116;

    tt=seconds(1)+0.05;
    while(seconds(1)<tt){go_front(120);samp();}
     sound(1000,1000);
     while(gd_f[0]>435){go2(150,150);samp();}
     while(gd_r>60){go2(120,120);samp();}
   while(gd_b[0]>440){go2(100,100);samp();}
  //  while(gd_l<gd_threshold_l)  {go_front(120);samp();}
  //      tt=seconds(1)+0.4;
  //  while(seconds(1)<tt){motor(1,100);motor(2,100);samp();}
    motor(1,0);motor(2,0);
wait(0.2);  
comturn_hh(165,-1,0,0);
    motor(1,0);motor(2,0);
     
     
   

  
    
    }
  
   
    }
    else if(pcs==9)
    {
    while(1)
    {
    led=0;
    test_port2();
    }
    }
    else if(pcs==10)
    {test_adc();
    }
	
    motor(1,0);motor(2,0);
}





























