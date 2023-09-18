/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) quadrotor_tailsitter_hybrid_expl_ode_fun_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[3] = {0, 0, 0};

/* quadrotor_tailsitter_hybrid_expl_ode_fun:(i0[8],i1[3],i2[])->(o0[8]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][3] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a1=arg[0]? arg[0][4] : 0;
  if (res[0]!=0) res[0][1]=a1;
  a2=arg[0]? arg[0][5] : 0;
  if (res[0]!=0) res[0][2]=a2;
  a3=9.8100000000000005e+00;
  a4=arg[0]? arg[0][7] : 0;
  a5=sin(a4);
  a6=arg[0]? arg[0][6] : 0;
  a7=cos(a6);
  a5=(a5*a7);
  a7=arg[1]? arg[1][0] : 0;
  a5=(a5*a7);
  a8=2.0000000000000001e-01;
  a5=(a5/a8);
  a5=(a3*a5);
  a9=6.7381541265540634e-01;
  a10=-6.4818253218932176e-01;
  a11=2.;
  a12=1.;
  a13=-2.;
  a14=-1.0418058405523500e+00;
  a15=6.7390468584570939e-01;
  a16=3.6749826614590775e-01;
  a17=-2.5419756403955609e+00;
  a0=(a0-a17);
  a16=(a16*a0);
  a0=-1.;
  a16=(a16+a0);
  a15=(a15*a16);
  a17=7.0806164009666481e-02;
  a18=3.1102272652105034e-01;
  a19=-3.0739945116325633e+00;
  a1=(a1-a19);
  a18=(a18*a1);
  a18=(a18+a0);
  a17=(a17*a18);
  a15=(a15+a17);
  a17=3.6157493138990145e-02;
  a1=3.2234973060256750e-01;
  a19=-2.6526598452989125e+00;
  a2=(a2-a19);
  a1=(a1*a2);
  a1=(a1+a0);
  a17=(a17*a1);
  a15=(a15+a17);
  a17=8.0330649426165412e-02;
  a2=1.5572080788488301e+00;
  a19=-6.4461399335662439e-01;
  a19=(a6-a19);
  a2=(a2*a19);
  a2=(a2+a0);
  a17=(a17*a2);
  a15=(a15+a17);
  a17=2.7900118429133164e-01;
  a19=1.5737721243006857e+00;
  a20=-6.2281300147222995e-01;
  a20=(a4-a20);
  a19=(a19*a20);
  a19=(a19+a0);
  a17=(a17*a19);
  a15=(a15+a17);
  a17=-3.2772000597057194e-01;
  a20=4.8636135576549764e+00;
  a21=8.0975898500000004e-02;
  a21=(a7-a21);
  a20=(a20*a21);
  a20=(a20+a0);
  a17=(a17*a20);
  a15=(a15+a17);
  a14=(a14+a15);
  a14=(a13*a14);
  a14=exp(a14);
  a14=(a12+a14);
  a14=(a11/a14);
  a14=(a14-a12);
  a10=(a10*a14);
  a15=3.1229750595058342e-01;
  a17=1.0438882908335985e+00;
  a21=7.7735331301216215e-01;
  a21=(a21*a16);
  a22=7.1683426986110119e-02;
  a22=(a22*a18);
  a21=(a21+a22);
  a22=-3.8296451822945748e-02;
  a22=(a22*a1);
  a21=(a21+a22);
  a22=-1.4899050423744642e-01;
  a22=(a22*a2);
  a21=(a21+a22);
  a22=-4.5617172083628982e-01;
  a22=(a22*a19);
  a21=(a21+a22);
  a22=-5.4434931526737895e-02;
  a22=(a22*a20);
  a21=(a21+a22);
  a17=(a17+a21);
  a17=(a13*a17);
  a17=exp(a17);
  a17=(a12+a17);
  a17=(a11/a17);
  a17=(a17-a12);
  a15=(a15*a17);
  a10=(a10+a15);
  a15=5.5938196508265337e-01;
  a21=-7.1768119190104063e-02;
  a22=3.6549625536627559e+00;
  a22=(a22*a16);
  a23=-1.9127067963648312e+00;
  a23=(a23*a18);
  a22=(a22+a23);
  a23=-3.0313389798635354e-01;
  a23=(a23*a1);
  a22=(a22+a23);
  a23=-2.2532400039835715e-01;
  a23=(a23*a2);
  a22=(a22+a23);
  a23=-1.6776767539119928e+00;
  a23=(a23*a19);
  a22=(a22+a23);
  a23=-5.9167174284024460e-01;
  a23=(a23*a20);
  a22=(a22+a23);
  a21=(a21+a22);
  a21=(a13*a21);
  a21=exp(a21);
  a21=(a12+a21);
  a21=(a11/a21);
  a21=(a21-a12);
  a15=(a15*a21);
  a10=(a10+a15);
  a15=1.0911894230553389e+00;
  a22=-1.0054076660159357e-01;
  a23=-2.1661860762469858e+00;
  a23=(a23*a16);
  a24=9.5480141696691934e-01;
  a24=(a24*a18);
  a23=(a23+a24);
  a24=1.6037600219822920e-01;
  a24=(a24*a1);
  a23=(a23+a24);
  a24=4.0918259404041202e-02;
  a24=(a24*a2);
  a23=(a23+a24);
  a24=1.0325209219370137e+00;
  a24=(a24*a19);
  a23=(a23+a24);
  a24=3.0367695080955270e-01;
  a24=(a24*a20);
  a23=(a23+a24);
  a22=(a22+a23);
  a22=(a13*a22);
  a22=exp(a22);
  a22=(a12+a22);
  a22=(a11/a22);
  a22=(a22-a12);
  a15=(a15*a22);
  a10=(a10+a15);
  a15=1.5007027450744239e+00;
  a23=-1.3104795492642938e+00;
  a24=-4.8081904381400281e-01;
  a24=(a24*a16);
  a16=-2.5587763238074918e-01;
  a16=(a16*a18);
  a24=(a24+a16);
  a16=-7.0144045767730825e-03;
  a16=(a16*a1);
  a24=(a24+a16);
  a16=3.0714092763533086e-02;
  a16=(a16*a2);
  a24=(a24+a16);
  a16=-3.3130059730204824e-02;
  a16=(a16*a19);
  a24=(a24+a16);
  a16=-1.2867874992031886e-01;
  a16=(a16*a20);
  a24=(a24+a16);
  a23=(a23+a24);
  a13=(a13*a23);
  a13=exp(a13);
  a13=(a12+a13);
  a11=(a11/a13);
  a11=(a11-a12);
  a15=(a15*a11);
  a10=(a10+a15);
  a9=(a9+a10);
  a9=(a9-a0);
  a10=1.4696908447743698e-01;
  a9=(a9/a10);
  a10=-6.5141567111656293e+00;
  a9=(a9+a10);
  a5=(a5+a9);
  if (res[0]!=0) res[0][3]=a5;
  a5=1.4542071954847786e+00;
  a9=-7.5526935481862734e-03;
  a9=(a9*a14);
  a10=-1.4386805341639155e+00;
  a10=(a10*a17);
  a9=(a9+a10);
  a10=-1.7859287370455379e-01;
  a10=(a10*a21);
  a9=(a9+a10);
  a10=-5.3003219232968624e-01;
  a10=(a10*a22);
  a9=(a9+a10);
  a10=-3.3210419554279440e-02;
  a10=(a10*a11);
  a9=(a9+a10);
  a5=(a5+a9);
  a5=(a5-a0);
  a9=2.0365303962857903e-01;
  a5=(a5/a9);
  a9=-7.1692571293151115e+00;
  a5=(a5+a9);
  a9=sin(a6);
  a9=(a9*a7);
  a9=(a9/a8);
  a9=(a3*a9);
  a5=(a5-a9);
  if (res[0]!=0) res[0][4]=a5;
  a5=-9.8100000000000005e+00;
  a9=cos(a4);
  a10=cos(a6);
  a9=(a9*a10);
  a9=(a9*a7);
  a9=(a9/a8);
  a3=(a3*a9);
  a5=(a5+a3);
  a3=1.8232314322334975e+00;
  a9=1.0946526044317371e+00;
  a9=(a9*a14);
  a14=2.1937929220976238e+00;
  a14=(a14*a17);
  a9=(a9+a14);
  a14=2.1506188428018194e-01;
  a14=(a14*a21);
  a9=(a9+a14);
  a14=5.8605084984991551e-01;
  a14=(a14*a22);
  a9=(a9+a14);
  a14=3.4287400461712907e+00;
  a14=(a14*a11);
  a9=(a9+a14);
  a3=(a3+a9);
  a3=(a3-a0);
  a0=1.5411492004438654e-01;
  a3=(a3/a0);
  a0=-4.7381157461440786e+00;
  a3=(a3+a0);
  a5=(a5+a3);
  if (res[0]!=0) res[0][5]=a5;
  a5=arg[1]? arg[1][1] : 0;
  a5=(a5-a6);
  a6=1.6669999999999999e-01;
  a5=(a5/a6);
  if (res[0]!=0) res[0][6]=a5;
  a5=arg[1]? arg[1][2] : 0;
  a5=(a5-a4);
  a5=(a5/a6);
  if (res[0]!=0) res[0][7]=a5;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_tailsitter_hybrid_expl_ode_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_tailsitter_hybrid_expl_ode_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_tailsitter_hybrid_expl_ode_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_tailsitter_hybrid_expl_ode_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_tailsitter_hybrid_expl_ode_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_tailsitter_hybrid_expl_ode_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_tailsitter_hybrid_expl_ode_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_tailsitter_hybrid_expl_ode_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_tailsitter_hybrid_expl_ode_fun_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_tailsitter_hybrid_expl_ode_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_tailsitter_hybrid_expl_ode_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_tailsitter_hybrid_expl_ode_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_tailsitter_hybrid_expl_ode_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_tailsitter_hybrid_expl_ode_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_tailsitter_hybrid_expl_ode_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_tailsitter_hybrid_expl_ode_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
