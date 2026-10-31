

module Render
using GLFW, ModernGL, LinearAlgebra
using StaticArrays
using JRTrack
include("classic3do.jl"); using .CLASSIC3DO
include("classicmip.jl"); using .CLASSICMip
include("classicdat.jl"); using .CLASSICDat

"""One renderable track mesh: interleaved 11-float verts, its diffuse texture
name (`""` if none), and a fallback colour (used untextured)."""
struct TrackPart; verts::Vector{Float32}; tex::String; col::NTuple{3,Float32}; end

"""Procedural wire-spoke wheel (untextured), axis along Z, 11-float verts."""
function wheel_mesh(r, hw; seg=28)
    out=Float32[]; tyre=(0.06f0,0.06f0,0.07f0); rim=(0.20f0,0.21f0,0.24f0)
    push3(p,n,c)=append!(out, Float32[p[1],p[2],p[3], n[1],n[2],n[3], c[1],c[2],c[3], 0f0,0f0])
    for i in 0:seg-1
        a0=2π*i/seg; a1=2π*(i+1)/seg
        for (rr,col) in ((r,tyre),(r*0.55f0,rim))
            x0,y0=rr*cos(a0),rr*sin(a0); x1,y1=rr*cos(a1),rr*sin(a1)
            n0=(cos(a0),sin(a0),0f0); n1=(cos(a1),sin(a1),0f0)
            push3((x0,y0,-hw),n0,col); push3((x1,y1,-hw),n1,col); push3((x1,y1,hw),n1,col)
            push3((x0,y0,-hw),n0,col); push3((x1,y1,hw),n1,col); push3((x0,y0,hw),n0,col)
            for z in (hw,-hw)
                push3((0f0,0f0,z),(0f0,0f0,Float32(sign(z))),col)
                push3((x0,y0,z),(0f0,0f0,Float32(sign(z))),col); push3((x1,y1,z),(0f0,0f0,Float32(sign(z))),col)
            end
        end
    end
    out
end

const M4 = SMatrix{4,4,Float32,16}
ident() = M4(1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1)
const _U4 = Matrix{Float32}(undef, 4, 4)
"""PERF-3: a Matrix{Float32} GL can take a pointer to, without allocating (single render thread)."""
@inline umat(M::Matrix{Float32}) = M
@inline umat(M::AbstractMatrix) = (copyto!(_U4, M); _U4)
const NDRAW = Ref(0); const NTRI = Ref(0)
const IDENT = Matrix{Float32}(I,4,4)
"""PERF-1: T(t) * Ry(ay) * Rz(az) * Rx(ax) * T(o) in ONE allocation -- the AI car transforms were built from
four matrices and three products per body PART per pass (the top allocation site, >1 MB/frame)."""
function pose_matrix(tx, ty, tz, ay, az, ax, ox = 0f0, oy = 0f0, oz = 0f0)
    cy, sy = cos(Float32(ay)), sin(Float32(ay)); cz, sz = cos(Float32(az)), sin(Float32(az)); cx, sx = cos(Float32(ax)), sin(Float32(ax))

    r11 = cy*cz;  r12 = -cy*sz*cx + sy*sx;  r13 = cy*sz*sx + sy*cx
    r21 = sz;     r22 = cz*cx;              r23 = -cz*sx
    r31 = -sy*cz; r32 = sy*sz*cx + cy*sx;   r33 = -sy*sz*sx + cy*cx
    o1, o2, o3 = Float32(ox), Float32(oy), Float32(oz)
    M4(r11, r21, r31, 0f0,  r12, r22, r32, 0f0,  r13, r23, r33, 0f0,
       Float32(tx) + r11*o1 + r12*o2 + r13*o3, Float32(ty) + r21*o1 + r22*o2 + r23*o3,
       Float32(tz) + r31*o1 + r32*o2 + r33*o3, 1f0)
end

translate(t) = M4(1,0,0,0, 0,1,0,0, 0,0,1,0, t[1],t[2],t[3],1)
translate(x, y, z) = M4(1,0,0,0, 0,1,0,0, 0,0,1,0, x,y,z,1)
function roty(a)
    c,s = cos(a),sin(a); M4(c,0,-s,0, 0,1,0,0, s,0,c,0, 0,0,0,1)
end
function rotz(a)
    c,s = cos(a),sin(a); M4(c,s,0,0, -s,c,0,0, 0,0,1,0, 0,0,0,1)
end
function rotx(a)
    c,s = cos(a),sin(a); M4(1,0,0,0, 0,c,s,0, 0,-s,c,0, 0,0,0,1)
end
scalexyz(x,y,z) = M4(x,0,0,0, 0,y,0,0, 0,0,z,0, 0,0,0,1)

function disc_normal(v)
    n=length(v)÷11; nx=ny=nz=0.0
    for k in 0:n-1; nx+=v[k*11+4]; ny+=v[k*11+5]; nz+=v[k*11+6]; end
    Float32.(normalize([nx,ny,nz]))
end

function rotaxis(axis, θ)
    x,y,z = normalize(SVector{3,Float64}(axis[1], axis[2], axis[3])); c=cos(θ); s=sin(θ); t=1-c
    M4(t*x*x+c, t*x*y+s*z, t*x*z-s*y, 0,
       t*x*y-s*z, t*y*y+c, t*y*z+s*x, 0,
       t*x*z+s*y, t*y*z-s*x, t*z*z+c, 0,  0, 0, 0, 1)
end
function perspective(fovy,aspect,near,far)
    f=1/tan(fovy/2)
    M4(f/aspect,0,0,0, 0,f,0,0, 0,0,(far+near)/(near-far),-1, 0,0,2*far*near/(near-far),0)
end

function perspective_revz(fovy,aspect,near,far)
    f=1/tan(fovy/2)
    M4(f/aspect,0,0,0, 0,f,0,0, 0,0,near/(far-near),-1, 0,0,far*near/(far-near),0)
end
function lookat(eye,ctr,up)
    e = SVector{3,Float64}(eye[1], eye[2], eye[3]); c = SVector{3,Float64}(ctr[1], ctr[2], ctr[3])
    f=normalize(c .- e); s=normalize(cross(f, SVector{3,Float64}(up[1], up[2], up[3]))); u=cross(s,f)
    M4(s[1],u[1],-f[1],0, s[2],u[2],-f[2],0, s[3],u[3],-f[3],0, -dot(s,e),-dot(u,e),dot(f,e),1)
end

const HORIZON = (0.78f0, 0.78f0, 0.75f0)
const ZENITH  = (0.40f0, 0.56f0, 0.78f0)
const VSRC = """
#version 330 core
layout(location=0) in vec3 pos; layout(location=1) in vec3 nrm;
layout(location=2) in vec3 col; layout(location=3) in vec2 uv;
uniform mat4 uVP; uniform mat4 uModel; uniform mat4 uLightVP;
uniform vec4 uClip[4]; uniform int uNClip;   // CLASSICVIS-1 S4: Classic's per-cell clip planes (world), 0 = none
out float gl_ClipDistance[4];
out vec3 vN; out vec3 vC; out vec2 vUV; out vec3 vWorld; out vec4 vLS;
void main(){ vN=mat3(uModel)*nrm; vC=col; vUV=uv;
  vWorld=(uModel*vec4(pos,1.0)).xyz; vLS=uLightVP*vec4(vWorld,1.0);
  for(int i=0;i<4;i++) gl_ClipDistance[i] = i < uNClip ? dot(uClip[i], vec4(vWorld,1.0)) : 1.0;
  gl_Position=uVP*uModel*vec4(pos,1.0); }"""
const FSRC = """
#version 330 core
in vec3 vN; in vec3 vC; in vec2 vUV; in vec3 vWorld; in vec4 vLS; out vec4 o;
uniform vec3 uLightDir; uniform sampler2D uTex; uniform int uHasTex; uniform float uBright;
uniform vec3 uCamPos; uniform vec3 uFogCol; uniform float uFogNear; uniform float uFogFar;
uniform sampler2D uShadow; uniform float uShadowTexel; uniform float uSpec; uniform int uBackFlip; uniform int uMirrorPass;
uniform float uWet;       // WETLOOK-1: 0 dry .. 1 wet road (sheen: darker asphalt, grazing sky reflection, soft glint)
uniform float uAmbFill;   // flat fill light (Classic pre-lit cockpit interior — lifts self-shadowed faces)
uniform int uFlatOn; uniform vec2 uFlat;   // SHADE-1: Classic-like near-flat object light: base*(uFlat.x + uFlat.y*sun)
uniform float uAlpha;     // per-draw opacity multiplier (1 = opaque; <1 = glass, e.g. the windscreen)
uniform int uCutout;      // 1 for chain-link/foliage cutouts → sharpen alpha edge (kill shimmer)
uniform int uGraze;       // 1 for Classic tree-LINE meshes → fade faces viewed edge-on (kills the end-on "smear")
uniform int uSky;
uniform float uMaxDist;   // TRACKS-TD-1: > 0 discards fragments farther than this from the camera (the track mesh's draw range)
// E69-S8 WHITE BALANCE. Gold renders neutral surfaces slightly COOL (asphalt R-B -2.5..-5.3 across
// Monza/Spa/Zandvoort) while native renders them WARM (+7.8..+12.7 across Zandvoort/Monza/Watkins).
// Decode is NOT the cause: the .mip header's own declared average colour for asphalt is (143,137,132),
// R-B +11, and our decode reproduces that hue — the texture really is authored warm, and gold cools
// it. Nor is the ambient fill (removing it entirely left R-B unchanged). So the correction belongs at
// the end of shading, as a white balance derived from the measurement rather than from taste.
uniform vec3 uWBal;
// E72-S13 PER-TRACK EXPOSURE. Measured on asphalt — a neutral surface needing no location matching —
// with COCKPIT view on both sides (E72-S12 showed view and source mismatch both corrupt this):
//   gold / native asphalt luminance:  Spa 120.2/149.0  Watkins 120.0/143.8  Monza 117.5/101.3
//                                     Zandvoort 128.5/131.3  Nurburgring 104.0/105.7
// Three tracks are off by ~20%. JM_EXPOSURE overrides; 1.0 disables.
uniform float uExposure;
uniform vec3 uSunCol;     // directional-sun tint (warm white on the sunny grade; white = neutral Classic)
uniform vec3 uAmbSky;     // up-facing sky-fill colour (cool blue on the sunny grade → cooler shadows)
uniform float uSat;       // output saturation multiplier (>1 = punchier sunny colours; 1 = neutral)
uniform int uSkyKey;      // HORIZ3DO-1: key out the panels' near-white haze rows (tracks whose horiz.3do has no sky)
uniform vec3 uSkyTint;    // Classic horizon-ring multiply (warm/brighten the overcast band toward sunny)
uniform vec3 uTint;       // per-draw colour multiply (default white = no-op; e.g. de-blue the crowd MIP)
uniform int uMirrorGlass; // 1 for the cockpit mirror glass quads → unlit round-masked RTT sample (E64)
uniform int uUnlit;       // E83-S3: 1 = sprite drawn at texture brightness (fog/WB/exposure only), as Classic draws its billboards
uniform int uMacro;       // 1 = macro tiling break-up on (default; JM_MACRO=0 disables — E68 S3 A/B)
// CARGOLD-2 S5: NORMAL-OFFSET lookup. The depth bias below is in light-space depth, i.e. METRES along the light ray
// (0.0018 x the 120 m box = 0.22 m): any occluder within it of the receiver casts nothing, so the bottom ~15 cm of every
// tyre threw no shadow and each tyre's shadow started a hand's width from its contact patch (the PO's "float").
// Offsetting the LOOKUP point along the surface normal by ~a shadow texel (uShadowNofs, m) kills flat-ground acne the
// way the big bias did, so the bias can shrink (uShadowBiasK). 0 / 1 = the pre-S5 shading exactly.
uniform mat4 uLightVP; uniform float uShadowNofs; uniform float uShadowBiasK;
float shadow(vec3 N){
  float ndl = clamp(dot(normalize(N), normalize(uLightDir)), 0.0, 1.0);
  vec4 ls = uShadowNofs > 0.0 ? uLightVP*vec4(vWorld + normalize(N)*uShadowNofs*(1.0 - ndl), 1.0) : vLS;
  vec3 lp = ls.xyz/ls.w*0.5+0.5;
  if(lp.z>1.0 || lp.x<0.0||lp.x>1.0||lp.y<0.0||lp.y>1.0) return 1.0;
  float bias = uShadowBiasK*max(0.0035*(1.0-dot(N,normalize(uLightDir))), 0.0018);   // larger → kill flat-ground acne
  float s=0.0;
  for(int x=-1;x<=1;x++) for(int y=-1;y<=1;y++)
    s += (lp.z-bias > texture(uShadow, lp.xy+vec2(x,y)*uShadowTexel).r) ? 0.0 : 1.0;
  s /= 9.0;
  // fade shadowing to fully-lit before the shadow-map box edge, so the box boundary
  // (which tracks the car) isn't a visible "light carpet" sweeping the ground
  vec2 e = abs(lp.xy - 0.5);
  // E68 S3 (PO: "white highlight in the track moving ~5 car lengths ahead of the car"):
  // the 0.40→0.5 fade band was narrow enough to read as a bright carpet sweeping the road
  // (measured +38 luma at Monza s=1500).  Fade over 0.28→0.5 = the same endpoint with less
  // than half the gradient, imperceptible on tarmac.
  return mix(s, 1.0, smoothstep(0.28, 0.5, max(e.x, e.y)));
}
void main(){
  float uvj = dFdx(vUV).x*dFdy(vUV).y - dFdx(vUV).y*dFdy(vUV).x;   // (before any discard: derivatives need the whole quad)
  if(uMaxDist > 0.0 && length(vWorld-uCamPos) > uMaxDist) discard;
  // un-mirror back-facing sign text -- TD261029-5 (3): only a back face whose texture actually READS mirrored. The sign of
  // the screen-space UV Jacobian says so (readable: u rightward, v downward -> negative; the mirror passes' X-flipped
  // projection inverts it). Zandvoort's grandstand front (gstand/gfront) has a panel wound away from the road whose UVs
  // still run along the stand: drawn as is it reads DUNLOP, flipped it read backwards (the PO's 2026-10-29 run).
  // uBackFlip==2 = the old rule (every back face flipped).
  bool mir = (uMirrorPass==1) ? (uvj < 0.0) : (uvj > 0.0);
  vec2 uv = ((uBackFlip==2 || (uBackFlip==1 && mir)) && !gl_FrontFacing) ? vec2(1.0-vUV.x, vUV.y) : vUV;
  vec4 t = uHasTex==1 ? texture(uTex,uv) : vec4(vC,1.0);
  if(uHasTex==1){
    if(uCutout==1){
      if(t.a < 0.02) discard;
      // CUTOUTS ONLY (chain-link/foliage): sharpen the alpha to a ~1px screen-space edge.
      // Distant cutouts shimmer because mipmapping softens their alpha to mid-grey and
      // alpha-to-coverage then dithers it per pixel; rescaling by the derivative keeps the
      // edge ~1px (smooth + stable) at any distance.  Gated to cutouts so blended overlays
      // (the racing groove) keep their soft alpha and don't harden into a black strip.
      t.a = clamp((t.a - 0.5) / max(fwidth(t.a), 1e-4) + 0.5, 0.0, 1.0);
      // Far away the wires are sub-pixel (fwidth huge → alpha stuck ~0.5, keeps crawling):
      // fade that UNRESOLVED partial alpha toward transparent with distance so it drops out
      // cleanly.  Solid alpha (signs) → pow(1.0,·)=1 stays; only the flickery mid-alpha goes.
      t.a = pow(t.a, 1.0 + smoothstep(140.0, 460.0, length(vWorld-uCamPos))*4.0);
    } else if(t.a < 0.04) discard;              // blended/opaque: plain soft alpha-to-coverage
  }
  if(uSky==1){                                            // HORIZ3DO-1: panels cut out their sky
    if(t.a < 0.5) discard;
    // the panels' upper rows are opaque near-white haze that Classic fogs into its own sky; ours is not
    // that colour, so key them out (bright AND unsaturated) and let our sky show through
    float mn = min(t.r, min(t.g, t.b)), mx = max(t.r, max(t.g, t.b));
    float key = smoothstep(0.80, 0.93, mn) * (1.0 - smoothstep(0.06, 0.14, mx - mn));
    if(uSkyKey==1 && key > 0.5) discard;           // only when the horizon has no Classic sky of its own
    o=vec4(t.rgb*uSkyTint, 1.0); return; }     // horizon ring: unlit, unfogged backdrop (tinted/brightened per grade)
  if(uMirrorGlass==1){                                    // E64 live mirror glass: round-masked rear-view RTT sample, unlit
    vec2 d=vC.xy-vec2(0.5);                               // vC.xy = disc-local 0..1 coords (colour attr repurposed by the glass quad)
    if(dot(d,d)>0.25) discard;                            // round glass on a round disc
    o=vec4(t.rgb*uTint, 1.0); return;
  }
  if(uUnlit==1){                                          // E83-S3: Classic billboards are pre-lit art. Lighting them again
    float fogu = clamp((length(vWorld-uCamPos)-uFogNear)/(uFogFar-uFogNear), 0.0, 1.0);   // doubled their brightness:
    vec3 cu = mix(t.rgb*uTint, uFogCol, fogu*fogu) * uWBal * uExposure;                    // tree textures decode to gold's
    o=vec4(cu, (uHasTex==1 ? t.a : 1.0)*uAlpha); return;                                 // (57,80,45) yet rendered ~(137,154,86).
  }
  vec3 N = dot(vN,vN) > 1e-6 ? normalize(vN) : vec3(0.0,1.0,0.0);  // guard zero/degenerate normals
  if(!gl_FrontFacing) N=-N;
  float diff=max(dot(N,normalize(uLightDir)),0.0)*shadow(N);
  vec3 grd=vec3(0.34,0.36,0.26);                             // ground-bounce fill (warm)
  vec3 amb=mix(grd,uAmbSky,0.5+0.5*N.y)*0.46;                // sky-fill: uAmbSky tints the up-facing fill (cool=sunny shadows)
  vec3 base = t.rgb * uTint;                                 // per-draw colour multiply (default white)
  if(uHasTex==1 && uMacro==1 && max(abs(vUV.x),abs(vUV.y)) > 3.0)   // tiling surface: gently break up the repeat
    base *= mix(vec3(1.0), texture(uTex, vUV*0.07).rgb * 1.7, 0.45);   // softer → no harsh light/dark patches (E68 S3: uMacro gates the A/B — suspect for the near-field bright pool)
  vec3 lit = pow(base*(amb+0.5*uAmbFill+diff*1.15*uSunCol)*uBright, vec3(0.94));   // stronger sun (0.95→1.15), tinted by uSunCol; gamma→neutral (0.85→0.94) ⇒ Classic contrast
  // SHADE-1 (PO 2026-10-17, "match Classic"): Classic draws placed objects almost at their texture colour -- a roof ~1.0x, walls
  // 0.9-1.1x whichever way they face (gold 260802 Spa t 92 vs the textures). The sky-tinted fill + strong sun above gave
  // up-facing roofs 1.2x in blue ("albino") and walls away from the sun 0.5x ("shaded sides too dark"). Neutral, flat-ish.
  if(uFlatOn==1) lit = base*(uFlat.x + uFlat.y*diff)*uBright;
  lit += uAmbFill*vec3(0.13,0.135,0.125);   // ADDITIVE fill: lifts pure-black cockpit parts
                                            // (tub/dash) to a visible dark grey, as Classic pre-lights them
  if(uSpec > 0.0){                               // Blinn-Phong sheen (painted/chrome bodywork)
    vec3 V = normalize(uCamPos - vWorld);
    float s = pow(max(dot(N, normalize(normalize(uLightDir)+V)), 0.0), 28.0) * uSpec * step(0.01, diff);
    lit += s * vec3(1.0, 0.97, 0.9);
  }
  if(uWet > 0.0){                                 // WETLOOK-1 (PO: "slight road sheen ... not as pronounced as in reference")
    vec3 V = normalize(uCamPos - vWorld);
    float fr = pow(1.0 - max(dot(N, V), 0.0), 5.0);                    // Schlick: only the road far ahead glistens
    lit *= 1.0 - 0.16*uWet;                                             // wet asphalt reads darker
    lit = mix(lit, uFogCol*0.95, clamp(0.55*fr, 0.0, 1.0)*uWet);        // the overcast sky in the film of water
    float g = pow(max(dot(N, normalize(normalize(uLightDir)+V)), 0.0), 60.0);
    lit += g * 0.22 * uWet * uSunCol * step(0.01, diff);               // a soft sun glint, no mirror
  }
  lit = mix(vec3(dot(lit, vec3(0.299,0.587,0.114))), lit, uSat);   // grade: punch up colour for the sunny look (uSat=1 → no-op)
  float fog = clamp((length(vWorld-uCamPos)-uFogNear)/(uFogFar-uFogNear), 0.0, 1.0);
  float alpha = uHasTex==1 ? t.a : 1.0;
  if(uGraze==1){                                  // tree-line mesh: fade quads seen edge-on (no streaky end-on smear)
    float g = abs(dot(N, normalize(uCamPos - vWorld)));   // 0 = edge-on, 1 = face-on
    alpha *= smoothstep(0.05, 0.32, g);
  }
  vec3 outc = mix(lit, uFogCol, fog*fog);
  outc *= uWBal;                                  // E69-S8: measured white balance (see below)
  outc *= uExposure;                              // E72-S13: per-track exposure, measured on asphalt
  o=vec4(outc, alpha*uAlpha);     // alpha → MSAA coverage (smooth cutout edges); uAlpha = glass
}"""

const SHADOW_ALPHA = get(ENV, "JM_SHADOW_ALPHA", "1") != "0"
const DEPTH_VS = """
#version 330 core
layout(location=0) in vec3 pos; layout(location=3) in vec2 uv; uniform mat4 uLightVP; uniform mat4 uModel; out vec2 vUV;
void main(){ vUV=uv; gl_Position=uLightVP*uModel*vec4(pos,1.0); }"""
const DEPTH_FS = """
#version 330 core
in vec2 vUV; uniform sampler2D uTex; uniform int uHasTex;
void main(){ if(uHasTex==1 && texture(uTex,vUV).a < 0.5) discard; }"""

const SKY_VS = """
#version 330 core
const vec2 P[3]=vec2[3](vec2(-1,-1),vec2(3,-1),vec2(-1,3));
out vec2 ndc; void main(){ ndc=P[gl_VertexID]; gl_Position=vec4(P[gl_VertexID],0,1); }"""
const SKY_FS = """
#version 330 core
in vec2 ndc; out vec4 o;
uniform mat4 uInvVP; uniform vec3 uCamPos; uniform vec3 uHorizon; uniform vec3 uZenith; uniform vec3 uLightDir; uniform float uCloud;
float hash(vec2 p){ return fract(sin(dot(p,vec2(127.1,311.7)))*43758.5453); }
float noise(vec2 p){ vec2 i=floor(p),f=fract(p); f=f*f*(3.0-2.0*f);
  return mix(mix(hash(i),hash(i+vec2(1,0)),f.x), mix(hash(i+vec2(0,1)),hash(i+vec2(1,1)),f.x), f.y); }
float fbm(vec2 p){ float v=0.0,a=0.55; for(int i=0;i<5;i++){ v+=a*noise(p); p=p*2.03+vec2(1.7,9.2); a*=0.5; } return v; }
void main(){
  vec4 wp=uInvVP*vec4(ndc,1.0,1.0); vec3 dir=normalize(wp.xyz/wp.w - uCamPos);
  float t=clamp(dir.y,0.0,1.0);
  vec3 col=mix(uHorizon, uZenith, sqrt(t));
  float sun=pow(max(dot(dir,normalize(uLightDir)),0.0), 200.0);
  if(dir.y > 0.01){                                  // procedural cloud layer
    vec2 cp = (uCamPos.xz + dir.xz/dir.y*1400.0)*0.0011;
    float c = fbm(cp);
    float cov = smoothstep(0.52,0.82,c) * smoothstep(0.015,0.22,dir.y) * uCloud;
    col = mix(col, mix(vec3(0.75,0.78,0.84), vec3(1.0), c), cov*0.9);
    sun *= (1.0 - cov*0.85);
  }
  o=vec4(col + vec3(1.0,0.95,0.8)*sun, 1.0);
}"""
compileok(s)=(r=Ref{GLint}(); glGetShaderiv(s,GL_COMPILE_STATUS,r); r[]!=0)
function compile(src,kind)
    s=glCreateShader(kind); glShaderSource(s,1,Ptr{GLchar}[pointer(src)],C_NULL); glCompileShader(s)
    compileok(s) || (buf=Vector{UInt8}(undef,2048); glGetShaderInfoLog(s,2048,C_NULL,buf); error("shader: ",String(buf)))
    s
end
function program()

    _fs = FSRC
    if get(ENV,"JM_NOSHADOW","0") != "0"
        _fs = replace(_fs, "float shadow(vec3 N){" => "float shadow(vec3 N){ return 1.0;")
        println("  [noshadow] shadow() forced to 1.0 (JM_NOSHADOW=1)")
    end

    if get(ENV,"JM_FLATLIT","0") != "0"
        _fs = replace(_fs, "float diff=max(dot(N,normalize(uLightDir)),0.0)*shadow(N);" =>
                           "float diff=1.0;   // JM_FLATLIT")
        println("  [flatlit] diffuse forced to 1.0 (JM_FLATLIT=1) -- texture only")
    end
    if get(ENV,"JM_NOTEX","0") != "0"
        _fs = replace(_fs, "vec4 t = uHasTex==1 ? texture(uTex,uv) : vec4(vC,1.0);" =>
                           "vec4 t = vec4(1.0);   // JM_NOTEX")
        println("  [notex] texture forced to white (JM_NOTEX=1) -- shading only")
    end
    p=glCreateProgram(); glAttachShader(p,compile(VSRC,GL_VERTEX_SHADER)); glAttachShader(p,compile(_fs,GL_FRAGMENT_SHADER)); glLinkProgram(p); p
end
function skyprogram()
    p=glCreateProgram(); glAttachShader(p,compile(SKY_VS,GL_VERTEX_SHADER)); glAttachShader(p,compile(SKY_FS,GL_FRAGMENT_SHADER)); glLinkProgram(p); p
end
function empty_vao(); v=Ref{GLuint}(); glGenVertexArrays(1,v); v[]; end

const _ULOC = Dict{UInt32,Dict{String,GLint}}()
@inline function uloc(prog, name::AbstractString)
    d = get!(Dict{String,GLint}, _ULOC, UInt32(prog))
    l = get(d, name, GLint(-2))
    l == -2 || return l
    l = glGetUniformLocation(prog, name); d[String(name)] = l; l
end
u3(prog,name,t)=glUniform3f(uloc(prog,name), Float32(t[1]),Float32(t[2]),Float32(t[3]))
"""Draw the gradient sky behind everything (depth test off, no depth write)."""
function draw_sky(skyprog, vao, invVP, campos, lightdir; cloud::Real=1.0, horizon=HORIZON, zenith=ZENITH)
    glDisable(GL_DEPTH_TEST); glDepthMask(GL_FALSE)
    glUseProgram(skyprog)
    glUniformMatrix4fv(uloc(skyprog,"uInvVP"),1,GL_FALSE,umat(invVP))
    u3(skyprog,"uCamPos",campos); u3(skyprog,"uHorizon",horizon); u3(skyprog,"uZenith",zenith); u3(skyprog,"uLightDir",lightdir)
    glUniform1f(uloc(skyprog,"uCloud"), Float32(cloud))
    glBindVertexArray(vao); glDrawArrays(GL_TRIANGLES,0,3)
    glDepthMask(GL_TRUE); glEnable(GL_DEPTH_TEST)
end

const FXAA_VS = """
#version 330 core
const vec2 P[3]=vec2[3](vec2(-1,-1),vec2(3,-1),vec2(-1,3));
out vec2 uv; void main(){ vec2 p=P[gl_VertexID]; uv=p*0.5+0.5; gl_Position=vec4(p,0,1); }"""
const FXAA_FS = """
#version 330 core
in vec2 uv; out vec4 o; uniform sampler2D uTex; uniform vec2 uInv;
float L(vec3 c){ return dot(c, vec3(0.299,0.587,0.114)); }
void main(){
  vec3 mC=texture(uTex,uv).rgb;
  vec3 nw=texture(uTex,uv+vec2(-1,-1)*uInv).rgb, ne=texture(uTex,uv+vec2(1,-1)*uInv).rgb;
  vec3 sw=texture(uTex,uv+vec2(-1, 1)*uInv).rgb, se=texture(uTex,uv+vec2(1, 1)*uInv).rgb;
  float lnw=L(nw),lne=L(ne),lsw=L(sw),lse=L(se),lm=L(mC);
  float lmin=min(lm,min(min(lnw,lne),min(lsw,lse))), lmax=max(lm,max(max(lnw,lne),max(lsw,lse)));
  if(lmax-lmin < 0.045){ o=vec4(mC,1.0); return; }       // no edge here
  vec2 dir=vec2(-((lnw+lne)-(lsw+lse)), ((lnw+lsw)-(lne+lse)));
  float red=max((lnw+lne+lsw+lse)*0.25*(1.0/8.0), 1.0/128.0);
  float rcp=1.0/(min(abs(dir.x),abs(dir.y))+red);
  dir=clamp(dir*rcp, vec2(-8.0), vec2(8.0))*uInv;
  vec3 a=0.5*(texture(uTex,uv+dir*(1.0/3.0-0.5)).rgb + texture(uTex,uv+dir*(2.0/3.0-0.5)).rgb);
  vec3 b=a*0.5 + 0.25*(texture(uTex,uv+dir*-0.5).rgb + texture(uTex,uv+dir*0.5).rgb);
  float lb=L(b);
  o=vec4((lb<lmin||lb>lmax)?a:b, 1.0);
}"""
function fxaa_program()
    p=glCreateProgram(); glAttachShader(p,compile(FXAA_VS,GL_VERTEX_SHADER)); glAttachShader(p,compile(FXAA_FS,GL_FRAGMENT_SHADER)); glLinkProgram(p); p
end

function make_scene_fbo(w, h; samples=4)
    ms=Ref{GLuint}(); glGenFramebuffers(1,ms); glBindFramebuffer(GL_FRAMEBUFFER, ms[])
    c=Ref{GLuint}(); glGenRenderbuffers(1,c); glBindRenderbuffer(GL_RENDERBUFFER, c[])
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_RGBA8, w, h)
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, c[])
    d=Ref{GLuint}(); glGenRenderbuffers(1,d); glBindRenderbuffer(GL_RENDERBUFFER, d[])
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_DEPTH_COMPONENT32F, w, h)
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, d[])
    rf=Ref{GLuint}(); glGenFramebuffers(1,rf); glBindFramebuffer(GL_FRAMEBUFFER, rf[])
    rt=Ref{GLuint}(); glGenTextures(1,rt); glBindTexture(GL_TEXTURE_2D, rt[])
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,w,h,0,GL_RGBA,GL_UNSIGNED_BYTE,C_NULL)
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR); glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE); glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE)
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, rt[], 0)
    glBindFramebuffer(GL_FRAMEBUFFER, 0)
    (ms[], rf[], rt[])
end

function make_mirror_fbo(w, h)
    fbo=Ref{GLuint}(); glGenFramebuffers(1,fbo); glBindFramebuffer(GL_FRAMEBUFFER, fbo[])
    tex=Ref{GLuint}(); glGenTextures(1,tex); glBindTexture(GL_TEXTURE_2D, tex[])
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,w,h,0,GL_RGBA,GL_UNSIGNED_BYTE,C_NULL)
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR); glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE); glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE)
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex[], 0)
    d=Ref{GLuint}(); glGenRenderbuffers(1,d); glBindRenderbuffer(GL_RENDERBUFFER, d[])
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32F, w, h)
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, d[])
    glBindFramebuffer(GL_FRAMEBUFFER, 0)
    (fbo[], tex[])
end

function resolve_and_fxaa(fxaaprog, vao, msfbo, resolvefbo, resolvetex, w, h)
    glBindFramebuffer(GL_READ_FRAMEBUFFER, msfbo); glBindFramebuffer(GL_DRAW_FRAMEBUFFER, resolvefbo)
    glBlitFramebuffer(0,0,w,h, 0,0,w,h, GL_COLOR_BUFFER_BIT, GL_NEAREST)
    glBindFramebuffer(GL_FRAMEBUFFER, 0); glViewport(0,0,w,h)
    glDisable(GL_DEPTH_TEST)
    glUseProgram(fxaaprog); glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, resolvetex)
    glUniform1i(uloc(fxaaprog,"uTex"), 0)
    glUniform2f(uloc(fxaaprog,"uInv"), 1f0/w, 1f0/h)
    glBindVertexArray(vao); glDrawArrays(GL_TRIANGLES,0,3)
    glEnable(GL_DEPTH_TEST)
end

function depthprogram()
    p=glCreateProgram(); glAttachShader(p,compile(DEPTH_VS,GL_VERTEX_SHADER)); glAttachShader(p,compile(DEPTH_FS,GL_FRAGMENT_SHADER)); glLinkProgram(p); p
end
function ortho(l,r,b,t,n,f)
    M4(2/(r-l),0,0,0, 0,2/(t-b),0,0, 0,0,-2/(f-n),0, -(r+l)/(r-l),-(t+b)/(t-b),-(f+n)/(f-n),1)
end
const SHADOW_SIZE = 2048
function make_shadow_fbo(size=SHADOW_SIZE)
    fbo=Ref{GLuint}(); glGenFramebuffers(1,fbo)
    tex=Ref{GLuint}(); glGenTextures(1,tex); glBindTexture(GL_TEXTURE_2D,tex[])
    glTexImage2D(GL_TEXTURE_2D,0,GL_DEPTH_COMPONENT24,size,size,0,GL_DEPTH_COMPONENT,GL_FLOAT,C_NULL)
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST); glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST)
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_BORDER); glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_BORDER)
    glTexParameterfv(GL_TEXTURE_2D,GL_TEXTURE_BORDER_COLOR,Float32[1,1,1,1])
    glBindFramebuffer(GL_FRAMEBUFFER,fbo[]); glFramebufferTexture2D(GL_FRAMEBUFFER,GL_DEPTH_ATTACHMENT,GL_TEXTURE_2D,tex[],0)
    glDrawBuffer(GL_NONE); glReadBuffer(GL_NONE); glBindFramebuffer(GL_FRAMEBUFFER,0)
    (fbo[], tex[])
end
"""Light view-projection: an orthographic box from the sun direction, centred on
`center` (the car), so the shadow map tracks the action."""

function light_vp(center, lightdir;
                  R=parse(Float64, get(ENV, "JM_SHADOW_R", "70.0")),
                  depth=parse(Float64, get(ENV, "JM_SHADOW_DEPTH", "120.0")))
    L=normalize(Float64.(lightdir)); eye=Float64.(center) .+ L.*(depth/2)
    up = abs(L[2])>0.99 ? [0.0,0.0,1.0] : [0.0,1.0,0.0]
    ortho(-R,R,-R,R,1.0,depth) * lookat(eye, Float64.(center), up)
end

"""CLASSICVIS-1 S4: set up to four world-space clip planes (vec4 a,b,c,d: inside where a·x+b·y+c·z+d >= 0) for the next draws;
`clip_planes!(prog, nothing)` turns clipping off again."""
function clip_planes!(prog, planes)
    glUseProgram(prog)
    if planes === nothing
        for i in 0:3; glDisable(GL_CLIP_DISTANCE0 + i); end
        glUniform1i(uloc(prog,"uNClip"),0); return
    end
    n = min(length(planes), 4)
    for i in 0:3; glEnable(GL_CLIP_DISTANCE0 + i); end
    buf = zeros(Float32, 16); for i in 1:n, j in 1:4; buf[4(i-1)+j] = Float32(planes[i][j]); end
    glUniform4fv(uloc(prog,"uClip[0]"), 4, buf); glUniform1i(uloc(prog,"uNClip"), n)
end
const SHADOW_NOFS  = parse(Float32, get(ENV, "JM_SHADOW_NOFS", "0.07"))
const SHADOW_BIASK = parse(Float32, get(ENV, "JM_SHADOW_BIASK", "0.15"))
const SHADOW_POFS = Tuple(parse.(Float32, split(get(ENV, "JM_SHADOW_POFS", "2.5,4.0"), ",")))
function shadow_pass(drawfn, depthprog, fbo, lightVP; size=SHADOW_SIZE)

    glClipControl(GL_LOWER_LEFT, GL_NEGATIVE_ONE_TO_ONE); glDepthFunc(GL_LESS); glClearDepth(1.0)
    glBindFramebuffer(GL_FRAMEBUFFER,fbo); glViewport(0,0,size,size); glClear(GL_DEPTH_BUFFER_BIT)
    glUseProgram(depthprog); glUniformMatrix4fv(uloc(depthprog,"uLightVP"),1,GL_FALSE,umat(lightVP))
    glEnable(GL_POLYGON_OFFSET_FILL); glPolygonOffset(SHADOW_POFS[1], SHADOW_POFS[2])
    drawfn(depthprog)
    glDisable(GL_POLYGON_OFFSET_FILL); glBindFramebuffer(GL_FRAMEBUFFER,0)
end

negdet(m) = (m[1,1]*(m[2,2]*m[3,3] - m[2,3]*m[3,2]) - m[1,2]*(m[2,1]*m[3,3] - m[2,3]*m[3,1]) +
             m[1,3]*(m[2,1]*m[3,2] - m[2,2]*m[3,1])) < 0
function draw_depth(depthprog, item, model)
    glUniformMatrix4fv(uloc(depthprog,"uModel"),1,GL_FALSE, umat(model))
    if SHADOW_ALPHA && item.tex != 0
        glUniform1i(uloc(depthprog,"uHasTex"), 1); glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, item.tex)
    else
        glUniform1i(uloc(depthprog,"uHasTex"), 0)
    end
    flip = negdet(model); flip && glFrontFace(GL_CW)
    glBindVertexArray(item.vao); glDrawArrays(GL_TRIANGLES,0,item.n)
    flip && glFrontFace(GL_CCW)
    NDRAW[] += 1; NTRI[] += item.n ÷ 3
end
"""Bind the shadow map + light matrix for the main pass (shadow on texture unit 1)."""
function bind_shadow(prog, shadowtex, lightVP; unit=1, size=SHADOW_SIZE)
    glUseProgram(prog)
    glUniformMatrix4fv(uloc(prog,"uLightVP"),1,GL_FALSE,umat(lightVP))
    glActiveTexture(GL_TEXTURE0+unit); glBindTexture(GL_TEXTURE_2D,shadowtex)
    glUniform1i(uloc(prog,"uShadow"),Int32(unit))
    glUniform1f(uloc(prog,"uShadowTexel"),Float32(1/size))
    glUniform1f(uloc(prog,"uShadowNofs"),SHADOW_NOFS); glUniform1f(uloc(prog,"uShadowBiasK"),SHADOW_BIASK)
end

"""Per-frame scene uniforms (camera position + distance fog into the haze)."""
function set_scene_uniforms(prog, campos; fognear=300f0, fogfar=2400f0,
                            fogcol=HORIZON, suncol=(1f0,1f0,1f0),
                            ambsky=(0.95f0,0.93f0,0.86f0), sat=1f0)
    glUseProgram(prog)
    u3(prog,"uCamPos",campos); u3(prog,"uFogCol",fogcol)
    u3(prog,"uSunCol",suncol); u3(prog,"uAmbSky",ambsky)
    glUniform1f(uloc(prog,"uSat"),Float32(sat))
    glUniform1f(uloc(prog,"uFogNear"),Float32(fognear))
    glUniform1f(uloc(prog,"uFogFar"),Float32(fogfar))
    glUniform3f(uloc(prog,"uTint"),1f0,1f0,1f0)
    glUniform1i(uloc(prog,"uMirrorGlass"),0)
    glUniform1i(uloc(prog,"uNClip"),0)
    glUniform1f(uloc(prog,"uMaxDist"),0f0)
    glUniform1i(uloc(prog,"uMacro"), get(ENV,"JM_MACRO","1")=="0" ? 0 : 1)
end

const HUD_VS = """
#version 330 core
layout(location=0) in vec2 p; layout(location=1) in vec3 c; uniform vec2 uRes; out vec3 col;
void main(){ col=c; gl_Position=vec4(p.x/uRes.x*2.0-1.0, 1.0-p.y/uRes.y*2.0, 0.0, 1.0); }"""
const HUD_FS = "#version 330 core\nin vec3 col; out vec4 o; void main(){ o=vec4(col,1.0); }"
function hud_program()
    p=glCreateProgram(); glAttachShader(p,compile(HUD_VS,GL_VERTEX_SHADER)); glAttachShader(p,compile(HUD_FS,GL_FRAGMENT_SHADER)); glLinkProgram(p); p
end
function hud_buffers(); vao=Ref{GLuint}();glGenVertexArrays(1,vao); vbo=Ref{GLuint}();glGenBuffers(1,vbo); (vao[],vbo[]); end
function hquad!(v,x,y,w,h,c)
    for (px,py) in ((x,y),(x+w,y),(x+w,y+h),(x,y),(x+w,y+h),(x,y+h)); push!(v, px, py, c[1], c[2], c[3]); end
end
const SEG7 = (63,6,91,79,102,109,125,7,127,111)
function hdigit!(v,x,y,W,H,T,d,c)
    m = d < 0 ? 84 : (0<=d<=9 ? SEG7[d+1] : return)
    vl=(H-3T)/2
    (m&1)!=0  && hquad!(v,x+T,y,W-2T,T,c)
    (m&32)!=0 && hquad!(v,x,y+T,T,vl,c)
    (m&2)!=0  && hquad!(v,x+W-T,y+T,T,vl,c)
    (m&64)!=0 && hquad!(v,x+T,y+T+vl,W-2T,T,c)
    (m&16)!=0 && hquad!(v,x,y+2T+vl,T,vl,c)
    (m&4)!=0  && hquad!(v,x+W-T,y+2T+vl,T,vl,c)
    (m&8)!=0  && hquad!(v,x+T,y+H-T,W-2T,T,c)
end
function hnumber!(v,x,y,W,H,T,n,c)
    s=string(max(0,Int(round(n))))
    for (i,ch) in enumerate(s); hdigit!(v,x+(i-1)*(W+2T),y,W,H,T,Int(ch)-48,c); end
end
function hline!(v,x1,y1,x2,y2,t,c)
    dx=x2-x1; dy=y2-y1; len=hypot(dx,dy); len<1e-3 && return
    px=-dy/len*(t/2); py=dx/len*(t/2)
    for (qx,qy) in ((x1+px,y1+py),(x2+px,y2+py),(x2-px,y2-py),(x1+px,y1+py),(x2-px,y2-py),(x1-px,y1-py))
        append!(v,Float32[qx,qy,c[1],c[2],c[3]])
    end
end
function hcircle!(v,cx,cy,r,t,c;seg=22)
    px,py=cx+r,cy
    for i in 1:seg; a=2π*i/seg; qx,qy=cx+r*cos(a),cy+r*sin(a); hline!(v,px,py,qx,qy,t,c); px,py=qx,qy; end
end
"Filled (SOLID) disc — triangle fan emitted as GL_TRIANGLES (center, p_i, p_i+1)."
function hdisc!(v,cx,cy,r,c;seg=26)
    for i in 1:seg
        a0=2π*(i-1)/seg; a1=2π*i/seg
        append!(v, Float32[cx,cy, c[1],c[2],c[3]])
        append!(v, Float32[cx+r*cos(a0),cy+r*sin(a0), c[1],c[2],c[3]])
        append!(v, Float32[cx+r*cos(a1),cy+r*sin(a1), c[1],c[2],c[3]])
    end
end

const GRIP_REF = 1.38
"""One per-wheel traction circle.  `tc` = (Fx_long, Fy_lat, GRIP=μ·Fz), all in mg/4.
The RING RADIUS scales with the grip (μ·Fz) — it GROWS as the tyre loads up under
weight transfer and SHRINKS as it goes light — and the dot is the force at the same
pixels-per-Newton scale, so it sits INSIDE the ring with grip in hand (green), at the
EDGE at the limit, and OUTSIDE when the tyre breaks away and skids (red)."""
function htraction!(v,cx,cy,baseR,tc; ring=(0.58,0.64,0.72), ringw=2.5)
    long,lat,grip = Float64(tc[1]),Float64(tc[2]),max(Float64(tc[3]),1e-3)
    s = baseR / GRIP_REF
    R = clamp(grip*s, baseR*0.45, baseR*1.7)
    util = hypot(long,lat)/grip
    col = util<0.7 ? (0.40,0.80,0.40) : util<0.95 ? (0.92,0.80,0.32) : (0.95,0.32,0.28)
    hdisc!(v,cx,cy,R,(0.09,0.10,0.13))
    hcircle!(v,cx,cy,R,ringw,ring)
    dx=-lat*s; dy=-long*s
    m=hypot(dx,dy); cap=R*1.5; m>cap && (dx*=cap/m; dy*=cap/m)
    hquad!(v,cx+dx-4,cy+dy-4,8,8,col)
end

function hellipse!(v,cx,cy,rx,ry,t,c;seg=28)
    px,py=cx+rx,cy
    for i in 1:seg; a=2π*i/seg; qx,qy=cx+rx*cos(a),cy+ry*sin(a); hline!(v,px,py,qx,qy,t,c); px,py=qx,qy; end
end
function htyre!(v,cx,cy,baseR,t)
    fx,fy,gx,gy,ξ = Float64(t[1]),Float64(t[2]),Float64(t[3]),Float64(t[4]),Float64(t[5])
    s = baseR / GRIP_REF
    rmax = 1.9*baseR
    hquad!(v,cx-rmax-3,cy-rmax-3,2rmax+6,2rmax+6,(0.09,0.10,0.13))
    hcircle!(v,cx,cy,baseR,1.0,(0.30,0.33,0.38))
    rx = clamp(gy*s, 2.0, rmax); ry = clamp(gx*s, 2.0, rmax)
    hellipse!(v,cx,cy,rx,ry,2.5,(0.62,0.68,0.76))
    col = ξ < 0.7 ? (0.40,0.80,0.40) : ξ < 1.0 ? (0.92,0.80,0.32) : (0.95,0.32,0.28)
    dx = -fy*s; dy = -fx*s
    m = hypot(dx,dy); m > rmax && (dx *= rmax/m; dy *= rmax/m)
    hquad!(v,cx+dx-4,cy+dy-4,8,8,col)
end
"""REPLAY-6: the four tyres as a 2×2 panel (FL FR / RL RR) centred at (cx, cy). Returns the cell pitch."""
function htyres!(v,cx,cy,baseR,ty)
    sp = 2*1.9*baseR + 12
    htyre!(v,cx-sp/2,cy-sp/2,baseR,ty[1]); htyre!(v,cx+sp/2,cy-sp/2,baseR,ty[2])
    htyre!(v,cx-sp/2,cy+sp/2,baseR,ty[3]); htyre!(v,cx+sp/2,cy+sp/2,baseR,ty[4])
    sp
end

"""Render a lap time `secs` as 7-segment M:SS.t at (x,y) in colour c."""
function htime!(v,x,y,secs,c; Wd=18,Hd=30,T=4,gap=6)
    secs = max(0.0, Float64(secs)); m = floor(Int, secs/60); s = secs - 60m
    si = floor(Int, s); ti = floor(Int, (s-si)*10); cx = Float64(x)
    hdigit!(v,cx,y,Wd,Hd,T, m%10, c); cx += Wd+gap
    hquad!(v,cx,y+Hd*0.28,T,T,c); hquad!(v,cx,y+Hd*0.60,T,T,c); cx += T+gap
    hdigit!(v,cx,y,Wd,Hd,T, si÷10, c); cx += Wd+3
    hdigit!(v,cx,y,Wd,Hd,T, si%10, c); cx += Wd+gap
    hquad!(v,cx,y+Hd-T,T,T,c); cx += T+gap
    hdigit!(v,cx,y,Wd,Hd,T, ti, c)
end

function hdial!(v,cx,cy,R,frac,redfrac,dimc,litc,redc)
    a0=Float32(deg2rad(225.0)); a1=Float32(deg2rad(-45.0))
    arc(f)=a0+(a1-a0)*clamp(Float32(f),0f0,1f0)
    n=30
    for i in 0:n
        f=i/n; θ=arc(f); c = f>=redfrac ? redc : dimc
        hquad!(v, cx+R*cos(θ)-1.6f0, cy-R*sin(θ)-1.6f0, 3.2f0, 3.2f0, c)
    end
    θ=arc(frac); nx=cx+(R-2f0)*cos(θ); ny=cy-(R-2f0)*sin(θ)
    bx=sin(θ)*2.3f0; by=cos(θ)*2.3f0
    append!(v, Float32[cx+bx,cy+by, litc[1],litc[2],litc[3]])
    append!(v, Float32[cx-bx,cy-by, litc[1],litc[2],litc[3]])
    append!(v, Float32[nx,ny,        litc[1],litc[2],litc[3]])
    hquad!(v, cx-2.5f0, cy-2.5f0, 5f0, 5f0, litc)
end

"""Compose the instrument readout: big 7-seg speed (bottom-left), gear (bottom-
right), throttle/brake/rpm bars, the four per-wheel traction circles (over the
nose), and last/best lap times (top-left).  `tc`=(FL,FR,RL,RR) (long,lat,radius)
or nothing; `lastlap`/`bestlap` in seconds (0 = none).  Returns the HUD vertex list."""
const _HUDV = Float32[]
function compose_hud(W,H,kmh,gear,rpm,revlim,thr,brk,clu=0.0,tc=nothing; lastlap=0.0, bestlap=0.0, manual=false, countdown=-1.0, clutchgate=-1.0, finished=0, startprompt=-1.0, band::Int=0)
    v=empty!(_HUDV)
    white=(0.90,0.95,1.0); amber=(1.0,0.82,0.35); green=(0.42,0.82,0.42); red=(0.95,0.35,0.30); dim=(0.16,0.18,0.22); blue=(0.35,0.65,1.0)

    if startprompt >= 0.0

        puls = 0.55 + 0.45*sin(2π*startprompt)
        lamp = (red[1]*puls, 0.08*puls, 0.08*puls)
        bw, bh = 300, 110
        bx, by = W÷2 - bw÷2, (band > 0 ? 10 : 24)
        hquad!(v, bx-6, by-6, bw+12, bh+12, dim)
        hquad!(v, bx, by, bw, bh, (0.07, 0.07, 0.08))
        for k in 0:2
            lx = bx + 30 + 90k; ly = by + 20
            hquad!(v, lx-4, ly-4, 78, 78, (0.18, 0.18, 0.2))
            hquad!(v, lx, ly, 70, 70, lamp)
        end
    end
    if countdown >= 0.0
        n = ceil(Int, countdown)
        if n > 0
            band > 0 ? hdigit!(v, W÷2 - 30, 12, 60, 110, 15, clamp(n, 0, 9), amber) :
                       hdigit!(v, W÷2 - 45, H÷2 - 90, 90, 170, 22, clamp(n, 0, 9), amber)
        else

            for k in 0:2; hquad!(v, W÷2 - 90 + 66k, (band > 0 ? 40 : H÷2 - 20), 48, 40, green); end
        end
    end
    if band == 0
    hnumber!(v, 40, H-104, 40, 76, 11, kmh, white)
    hdigit!(v, W-92, H-150, 40, 76, 11, (round(Int,gear) <= 0 ? -1 : clamp(round(Int,gear),1,9)), amber)
    hquad!(v, W-100, H-164, 12, 12, manual ? amber : green)

    let rc = rpm >= 0.95*revlim ? red : rpm >= 0.80*revlim ? amber : white
        hnumber!(v, 40, H-176, 26, 48, 7, round(Int, rpm), rc)
    end
    end

    if clutchgate >= 0.0
        bw, bh = 420, 26
        bx, by = W÷2 - bw÷2, (band > 0 ? band - 40 : H - 150)
        hquad!(v, bx-3, by-3, bw+6, bh+6, dim)
        hquad!(v, bx, by, bw, bh, (0.10,0.10,0.13))
        fill = round(Int, clamp(clutchgate, 0.0, 1.0) * bw)
        fill > 0 && hquad!(v, bx, by, fill, bh, red)
        tick = round(Int, 0.4 * bw)
        hquad!(v, bx + tick - 2, by - 8, 4, bh + 16, amber)
    end

    if finished > 0
        bw, bh = 480, 56
        bx, by = W÷2 - bw÷2, 60
        sq = 30
        hquad!(v, bx-4, by-4, bw+8, bh+8, dim)
        for k in 0:(bw ÷ sq - 1)
            top = iseven(k) ? white : (0.10,0.10,0.13)
            bot = iseven(k) ? (0.10,0.10,0.13) : white
            hquad!(v, bx + k*sq, by,        sq, bh÷2, top)
            hquad!(v, bx + k*sq, by + bh÷2, sq, bh÷2, bot)
        end
        hdigit!(v, W÷2 - 22, by + bh + 14, 44, 84, 12, clamp(finished, 0, 9), green)
    end
    lastlap > 0 && htime!(v, 40, 28, lastlap, white)
    bestlap > 0 && htime!(v, 40, 74, bestlap, green)
    v
end
function hud_draw(prog,vao,vbo,v,W,H)
    isempty(v) && return
    glDisable(GL_DEPTH_TEST); glUseProgram(prog)
    glUniform2f(uloc(prog,"uRes"),Float32(W),Float32(H))
    glBindVertexArray(vao); glBindBuffer(GL_ARRAY_BUFFER,vbo)
    glBufferData(GL_ARRAY_BUFFER,sizeof(v),v,GL_DYNAMIC_DRAW)
    glVertexAttribPointer(0,2,GL_FLOAT,false,5*4,Ptr{Cvoid}(0)); glEnableVertexAttribArray(0)
    glVertexAttribPointer(1,3,GL_FLOAT,false,5*4,Ptr{Cvoid}(2*4)); glEnableVertexAttribArray(1)
    glDrawArrays(GL_TRIANGLES,0,length(v)÷5); glEnable(GL_DEPTH_TEST)
end

"""Upload decoded RGBA pixels as a GL texture (GL builds the mips)."""

const CUTOUT_TEX = Dict{GLuint,Bool}()
function classify_cutout(rgba)
    N = length(rgba) ÷ 4; N == 0 && return false
    nt = 0; nmid = 0
    @inbounds for i in 4:4:length(rgba)
        a = rgba[i]
        a < 26 ? (nt += 1) : (a <= 230 && (nmid += 1))
    end
    nt/N > 0.05 && nmid/N < 0.22
end

is_groove_tex(name) = startswith(lowercase(name), "groove")
function soften_groove(w, h, rgba0)
    rgba = copy(rgba0); n = w*h
    a = Float32[rgba[4i+4] for i in 0:n-1]
    for _ in 1:something(tryparse(Int, get(ENV, "JM_GROOVE_BLUR", "3")), 3)
        b = similar(a)
        @inbounds for y in 0:h-1, x in 0:w-1
            s = 0f0
            for dy in -1:1, dx in -1:1
                s += a[mod(y + dy, h)*w + mod(x + dx, w) + 1]
            end
            b[y*w + x + 1] = s/9
        end
        a = b
    end
    k = something(tryparse(Float32, get(ENV, "JM_GROOVE_ALPHA", "0.6")), 0.6f0)
    g = UInt8(clamp(something(tryparse(Int, get(ENV, "JM_GROOVE_GREY", "48")), 48), 0, 255))
    @inbounds for i in 0:n-1
        rgba[4i+4] = round(UInt8, clamp(a[i+1]*k, 0f0, 255f0))
        for c in 1:3; rgba[4i+c] = max(rgba[4i+c], g); end
    end
    rgba
end
function upload_rgba(w, h, rgba)
    tex=Ref{GLuint}(); glGenTextures(1,tex); glBindTexture(GL_TEXTURE_2D,tex[])
    CUTOUT_TEX[tex[]] = classify_cutout(rgba)
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,w,h,0,GL_RGBA,GL_UNSIGNED_BYTE,rgba)
    glGenerateMipmap(GL_TEXTURE_2D)
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT); glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT)
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR); glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR)
    set_anisotropy()
    glTexParameterf(GL_TEXTURE_2D, GLenum(0x8501), 0f0)
    aniso=Ref{Float32}(0f0); glGetFloatv(GLenum(0x84FF),aniso)
    aniso[]>1 && glTexParameterf(GL_TEXTURE_2D,GLenum(0x84FE),min(aniso[],16f0))
    tex[]
end

is_crowd_texname(k) = occursin("crowd", k) || occursin("crwd", k) || occursin("sidecrd", k)
function mag_nearest(tid::GLuint)
    tid == 0 && return
    glBindTexture(GL_TEXTURE_2D, tid)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
end
function load_mip(path)
    w,h,rgba = CLASSICMip.decode_mip(path); upload_rgba(w,h,rgba)
end

struct CLASSICTex
    paths::Dict{String,String}
    dat::Dict{String,Vector{UInt8}}
    cachedir::String
end

"""Index a Classic folder's textures: loose .mip/.srb + every .dat archive's .mip/.srb."""
function classic_texture_index(dir)
    paths=Dict{String,String}(); dat=Dict{String,Vector{UInt8}}()
    for f in readdir(dir; join=true)
        lf=lowercase(f)
        (endswith(lf,".mip")||endswith(lf,".srb")) && (paths[lowercase(basename(f))[1:end-4]] = f)
        if endswith(lf,".dat")
            try; merge!(dat, CLASSICDat.parse_dat(f)); catch; end
        end
    end

    cd = get(ENV,"JM_TEXCACHE","1") == "0" ? "" :
         joinpath(get(ENV, "JM_TEXCACHE_DIR", joinpath(homedir(), ".cache", "juliamotor", "tex")),
                  string(hash((abspath(dir), get(ENV,"JM_NO_ALPHABLEED","0"), CLASSICMip.CMAP_ORDER, CLASSICDat.dat_layout())), base=16))
    cd != "" && (try; mkpath(cd); catch; cd = ""; end)
    CLASSICTex(paths, dat, cd)
end

function _alpha_bleed!(rgba::Vector{UInt8}, w::Int, h::Int)
    (w > 0 && h > 0 && length(rgba) >= 4*w*h) || return rgba
    @inline idxof(x, y) = 4*((y-1)*w + (x-1)) + 1
    for _pass in 1:4
        changed = false
        for y in 1:h, x in 1:w
            i = idxof(x,y)
            rgba[i+3] >= 0x80 && continue
            sr=0; sg=0; sb=0; n=0
            for dy in -1:1, dx in -1:1
                (dx==0 && dy==0) && continue
                xx=x+dx; yy=y+dy
                (xx<1 || yy<1 || xx>w || yy>h) && continue
                j = idxof(xx,yy)
                rgba[j+3] < 0x80 && continue
                sr += Int(rgba[j]); sg += Int(rgba[j+1]); sb += Int(rgba[j+2]); n += 1
            end
            if n > 0
                rgba[i]   = UInt8(sr ÷ n); rgba[i+1] = UInt8(sg ÷ n); rgba[i+2] = UInt8(sb ÷ n)
                rgba[i+3] = 0x01
                changed = true
            end
        end
        changed || break
    end
    rgba
end

const _TEXDECODED = Ref(0)

"""Resolve a texture name → (w,h,rgba) from the provider, or `nothing`.  E67: raw-blob cache."""
function tex_rgba(idx::CLASSICTex, name::AbstractString)
    key=lowercase(name)
    cf = idx.cachedir == "" ? "" : joinpath(idx.cachedir, key*".rgba")
    if cf != "" && isfile(cf)
        try
            io = open(cf); w = Int(read(io, Int32)); h = Int(read(io, Int32)); rgba = read(io); close(io)
            length(rgba) == 4*w*h && return (w, h, rgba)
        catch; end
    end
    r = _tex_rgba_decode(idx, key)

    if cf != "" && r !== nothing
        n = (_TEXDECODED[] += 1)
        n % 50 == 1 && (println("  [texcache] first run on this track: preparing textures, ", n,
                                " decoded so far (saved for later launches)"); flush(stdout))
    end

    if r !== nothing && get(ENV,"JM_NO_ALPHABLEED","0") == "0"
        _alpha_bleed!(r[3], r[1], r[2])
    end

    if r !== nothing && get(ENV,"JM_TEXCOLOR","") != ""
        want = ENV["JM_TEXCOLOR"]
        if want == "1" || occursin(lowercase(want), key)
            w,h,rgba = r
            n = w*h
            if n > 0
                sr=0; sg=0; sb=0; na=0
                for i in 1:4:length(rgba)-3
                    rgba[i+3] < 0x80 && continue
                    sr += Int(rgba[i]); sg += Int(rgba[i+1]); sb += Int(rgba[i+2]); na += 1
                end

                tr=0; tg=0; tb=0; nt=0
                for i in 1:4:length(rgba)-3
                    rgba[i+3] >= 0x80 && continue
                    tr += Int(rgba[i]); tg += Int(rgba[i+1]); tb += Int(rgba[i+2]); nt += 1
                end
                if na > 0
                    mr, mg, mb = sr/na, sg/na, sb/na
                    tstr = nt > 0 ? string("   TRANSPARENT (", round(Int,tr/nt), ",", round(Int,tg/nt),
                                           ",", round(Int,tb/nt), ") n=", nt) : "   (no transparent texels)"
                    println("  [texcolor] ", rpad(key,14), rpad(string(w,"x",h),10),
                            "opaque (", round(Int,mr), ",", round(Int,mg), ",", round(Int,mb),
                            ")   R-B = ", rpad(round(mr-mb, digits=1),7), tstr)
                    flush(stdout)
                end
            end
        end
    end
    if r !== nothing && cf != ""
        try; open(cf,"w") do io; write(io, Int32(r[1]), Int32(r[2]), r[3]); end; catch; end
    end
    return r
end
function _tex_rgba_decode(idx::CLASSICTex, key::AbstractString)
    p = get(idx.paths, key, "")
    if p != ""
        try
            return endswith(lowercase(p),".srb") ? CLASSICMip.decode_srb_bytes(read(p)) : CLASSICMip.decode_mip(p)
        catch; end
    end
    for (ext, dec) in ((".mip", CLASSICMip.decode_mip_bytes), (".srb", CLASSICMip.decode_srb_bytes))
        v = get(idx.dat, key*ext, nothing)
        if v !== nothing
            try; return dec(v); catch; end
        end
    end

    if length(key) >= 5
        for (k,pp) in idx.paths
            if startswith(k, key)
                try; return endswith(lowercase(pp),".srb") ? CLASSICMip.decode_srb_bytes(read(pp)) : CLASSICMip.decode_mip(pp); catch; end
            end
        end

        for (k,v) in idx.dat
            if startswith(k, key) && (endswith(k,".mip") || endswith(k,".srb"))
                try; return (endswith(k,".srb") ? CLASSICMip.decode_srb_bytes : CLASSICMip.decode_mip_bytes)(v); catch; end
            end
        end
    end
    nothing
end

"""Extract a Classic .3do car into textured TrackParts (one per texture), in the
car rig frame (X fwd, Y up, Z left).  Classic is X=fwd, Y=lateral, Z=up; V is flipped
(Classic 0=top, GL 0=bottom).  The reflection-tray / blob-shadow planes are dropped."""

function _clip_lat(t, maxlat::Float32)
    verts = [(t.p[i], t.n[i], t.uv[i]) for i in 1:3]
    for (sgn, lim) in ((1f0, maxlat), (-1f0, maxlat))
        isempty(verts) && break

        d(v) = sgn * v[1][2] - lim
        out = eltype(verts)[]
        n = length(verts)
        for i in 1:n
            a = verts[i]; b = verts[mod1(i+1, n)]
            da = d(a); db = d(b)
            da <= 0f0 && push!(out, a)
            if (da < 0f0) != (db < 0f0) && da != db
                w = da / (da - db)
                lerp3(u, v) = ntuple(k -> u[k] + w*(v[k]-u[k]), 3)
                lerp2(u, v) = ntuple(k -> u[k] + w*(v[k]-u[k]), 2)
                push!(out, (lerp3(a[1], b[1]), lerp3(a[2], b[2]), lerp2(a[3], b[3])))
            end
        end
        verts = out
    end
    length(verts) < 3 && return CLASSIC3DO.Tri[]
    length(verts) == 3 && verts[1][1] === t.p[1] && verts[2][1] === t.p[2] && verts[3][1] === t.p[3] &&
        return [t]
    [CLASSIC3DO.Tri((verts[1][1], verts[i][1], verts[i+1][1]),
                (verts[1][2], verts[i][2], verts[i+1][2]),
                (verts[1][3], verts[i][3], verts[i+1][3]),
                t.tex, t.col, t.flat, t.ptype) for i in 2:length(verts)-1]
end

function _triplane(t)
    a, b, c = t.p
    n = ((b[2]-a[2])*(c[3]-a[3]) - (b[3]-a[3])*(c[2]-a[2]), (b[3]-a[3])*(c[1]-a[1]) - (b[1]-a[1])*(c[3]-a[3]),
         (b[1]-a[1])*(c[2]-a[2]) - (b[2]-a[2])*(c[1]-a[1]))
    l = sqrt(n[1]^2 + n[2]^2 + n[3]^2); l < 1f-9 && return (0f0, 0f0, 0f0, 0f0)
    n = (n[1]/l, n[2]/l, n[3]/l)
    (Float32(n[1]), Float32(n[2]), Float32(n[3]), Float32(n[1]*a[1] + n[2]*a[2] + n[3]*a[3]))
end
function _inplane(t, planes; cosmin = 0.995f0, dtol = 0.03f0)
    q = _triplane(t); (q[1] == 0f0 && q[2] == 0f0 && q[3] == 0f0) && return false
    for pl in planes
        dot = q[1]*pl[1] + q[2]*pl[2] + q[3]*pl[3]
        abs(dot) >= cosmin && abs(q[4] - sign(dot)*pl[4]) <= dtol && return true
    end
    false
end
function extract_classic_car(path3do; exclude=("ltraymap","lshad"), only=(), grey=(0.72f0,0.74f0,0.76f0), smooth=true, tint=nothing, track=false, mirror=false, exclude_groups=(), include_groups=(), cockpit_clean=false, maxedge=Inf32, uflip=nothing, vflip=nothing, maxlat=Inf32, trim=false, dedup=nothing, drop_green=false, min_component=0, min_component_tex=(), wheel_dress=nothing, cockpit_dress=nothing, skip_ptypes_kw=nothing, visor=false, skin=nothing, visorcol=(0.75f0, 0.64f0, 0.22f0), flatcol=false, lappos::Float64=NaN, laplen::Float64=NaN, drop_unbound::Bool=false)
    livery = lowercase(first(splitext(basename(String(path3do)))))

    uflip === nothing && (uflip = false)
    vflip === nothing && (vflip = !mirror && (track || !CAR_NATIVE_UV))

    haskey(ENV,"JM_VFLIP") && (vflip = ENV["JM_VFLIP"] != "0")
    haskey(ENV,"JM_UFLIP") && (uflip = ENV["JM_UFLIP"] != "0")
    maxlat = Float32(maxlat)
    m = CLASSIC3DO.parse_3do(path3do; lappos, laplen)
    groups = Dict{String,Vector{Float32}}()
    edge(a,b) = sqrt((a[1]-b[1])^2+(a[2]-b[2])^2+(a[3]-b[3])^2)
    function triarea(p)
        ux=p[2][1]-p[1][1]; uy=p[2][2]-p[1][2]; uz=p[2][3]-p[1][3]
        vx=p[3][1]-p[1][1]; vy=p[3][2]-p[1][2]; vz=p[3][3]-p[1][3]
        0.5f0*sqrt((uy*vz-uz*vy)^2 + (uz*vx-ux*vz)^2 + (ux*vy-uy*vx)^2)
    end

    yellowish(c) = c[1]>0.55f0 && c[2]>0.40f0 && c[3]<0.40f0 && c[1] > c[3]+0.22f0

    istray(tex) = endswith(tex,"traymap") || endswith(tex,"shad")

    traygreen(c) = drop_green && c[2]>0.45f0 && c[1]<0.30f0 && c[3]<0.30f0
    _dropwide(t) = maxlat < Inf32 && max(abs(t.p[1][2]),abs(t.p[2][2]),abs(t.p[3][2])) > maxlat

    _wholly_out(t) = maxlat < Inf32 &&
        ((t.p[1][2] >  maxlat && t.p[2][2] >  maxlat && t.p[3][2] >  maxlat) ||
         (t.p[1][2] < -maxlat && t.p[2][2] < -maxlat && t.p[3][2] < -maxlat))
    overlat(t) = trim != false ? _wholly_out(t) : _dropwide(t)
    cmax(t) = max(abs(t.p[1][1]),abs(t.p[1][2]),abs(t.p[1][3]), abs(t.p[2][1]),abs(t.p[2][2]),abs(t.p[2][3]),
                  abs(t.p[3][1]),abs(t.p[3][2]),abs(t.p[3][3]))
    keep(t) = let L = max(edge(t.p[1],t.p[2]), edge(t.p[2],t.p[3]), edge(t.p[1],t.p[3])),
                  A = triarea(t.p)
        !isempty(only) && !(t.tex in only) ? false :
        !(isfinite(L) && isfinite(A) && cmax(t) < 5f4) ? false :
        (cockpit_clean && t.tex=="" && yellowish(t.col)) || overlat(t) ? false :
        track ? (!(t.tex in exclude) && A >= 1f-7 && L <= maxedge) :

                (!(t.tex in exclude) && !istray(t.tex) && !(t.tex=="" && traygreen(t.col)) && L <= min(2.0f0, maxedge) && A >= 1f-7 && !(A > 0 && L/(2A/L) > 200f0))
    end

    skip_ptypes = skip_ptypes_kw !== nothing ? skip_ptypes_kw : let e = get(ENV, "JM_SKIP_PTYPES", "")
        e == "0" || e == "" ? Set{UInt32}() : Set{UInt32}(parse(UInt32, strip(x)) for x in split(e, ","))
    end
    kept = [m.tris[i] for i in eachindex(m.tris) if keep(m.tris[i]) && !(m.groups[i] in exclude_groups) && !(m.tris[i].ptype in skip_ptypes) &&
            (isempty(include_groups) || m.groups[i] in include_groups)]

    drop_unbound && filter!(t -> !(t.tex == "" && !t.flat), kept)

    _dotrim(t) = trim isa Bool ? trim : (t.tex in trim)
    trim != false && maxlat < Inf32 &&
        (kept = collect(Iterators.flatten(_dotrim(t) ? _clip_lat(t, maxlat) :
                                          (_dropwide(t) ? CLASSIC3DO.Tri[] : [t]) for t in kept)))

    if min_component > 1 && !isempty(kept)
        qv(p) = (round(Int,p[1]*1000), round(Int,p[2]*1000), round(Int,p[3]*1000))
        parent = collect(1:length(kept))
        find(a) = (while parent[a] != a; parent[a] = parent[parent[a]]; a = parent[a]; end; a)
        union!(a, b) = (ra = find(a); rb = find(b); ra != rb && (parent[ra] = rb))
        seen = Dict{NTuple{3,Int},Int}()
        for (i, t) in enumerate(kept), k in (qv(t.p[1]), qv(t.p[2]), qv(t.p[3]))
            j = get(seen, k, 0)
            j == 0 ? (seen[k] = i) : union!(i, j)
        end
        count = Dict{Int,Int}()
        for i in 1:length(kept); r = find(i); count[r] = get(count, r, 0) + 1; end

        _mctex(i) = isempty(min_component_tex) || (kept[i].tex in min_component_tex)
        okc = Dict{Int,Bool}()
        for i in 1:length(kept); r = find(i); okc[r] = get(okc, r, true) && _mctex(i); end

        if haskey(ENV, "JM_MC_DIAG")
            for (r, n) in count
                (n < min_component && get(okc, r, false)) || continue
                idx = [i for i in 1:length(kept) if find(i) == r]
                lat = [abs(p[2]) for i in idx for p in kept[i].p]
                ys  = [p[3] for i in idx for p in kept[i].p]
                xs  = [p[1] for i in idx for p in kept[i].p]
                println("   [mc] dropping solid n=", n, "  x ", round(minimum(xs),digits=2), "..",
                        round(maximum(xs),digits=2), "  |lat| ", round(minimum(lat),digits=2), "..",
                        round(maximum(lat),digits=2), "  ydrop ", round(maximum(ys)-minimum(ys),digits=3),
                        "  tex ", join(unique([kept[i].tex for i in idx]), ","))
            end
        end
        kept = [t for (i, t) in enumerate(kept)
                if count[find(i)] >= min_component || !okc[find(i)]]
    end

    if cockpit_dress !== nothing && !isempty(kept)
        (cdtex, cdx0, cdx1, cdy0, cdz0) = cockpit_dress
        newk2 = CLASSIC3DO.Tri[]
        for t in kept
            if t.tex != ""; push!(newk2, t); continue; end
            cx = (t.p[1][1] + t.p[2][1] + t.p[3][1]) / 3
            cy = (t.p[1][3] + t.p[2][3] + t.p[3][3]) / 3
            cz = (t.p[1][2] + t.p[2][2] + t.p[3][2]) / 3
            inbox = (cdx0 <= cx <= cdx1) && (cy >= cdy0) && (abs(cz) >= cdz0)
            if inbox
                e1 = t.p[2] .- t.p[1]; e2 = t.p[3] .- t.p[1]
                nlat = e1[3]*e2[1] - e1[1]*e2[3]

                hasuv = let us=(t.uv[1][1],t.uv[2][1],t.uv[3][1]), vs=(t.uv[1][2],t.uv[2][2],t.uv[3][2])
                    (maximum(us)-minimum(us) > 1f-5) || (maximum(vs)-minimum(vs) > 1f-5)
                end

                _ = nlat
                if hasuv
                    push!(newk2, CLASSIC3DO.Tri(t.p, t.n, t.uv, cdtex, (1f0,1f0,1f0), false, t.ptype))
                    continue
                end
            end
            push!(newk2, t)
        end
        kept = newk2
    end

    if wheel_dress !== nothing && !isempty(kept)
        (typos, tyneg, tytread) = wheel_dress
        ys = Float32[p[2] for t in kept for p in t.p]
        y0, y1 = minimum(ys), max(maximum(ys), minimum(ys) + 1f-4)
        Rw = maximum(sqrt(p[1]^2 + p[3]^2) for t in kept for p in t.p)
        NREP = 4f0
        dressed = CLASSIC3DO.Tri[]
        for t in kept
            if t.tex != ""; push!(dressed, t); continue; end
            e1 = t.p[2] .- t.p[1]; e2 = t.p[3] .- t.p[1]
            nx = e1[2]*e2[3]-e1[3]*e2[2]; ny = e1[3]*e2[1]-e1[1]*e2[3]; nz = e1[1]*e2[2]-e1[2]*e2[1]
            l = sqrt(nx*nx + ny*ny + nz*nz)
            l < 1f-12 && (push!(dressed, t); continue)
            hasuv = let us = (t.uv[1][1], t.uv[2][1], t.uv[3][1]), vs = (t.uv[1][2], t.uv[2][2], t.uv[3][2])
                (maximum(us) - minimum(us) > 1f-5) || (maximum(vs) - minimum(vs) > 1f-5)
            end
            if abs(ny/l) > 0.55f0
                tex = ny > 0 ? typos : tyneg
                uv = hasuv ? t.uv :
                     ntuple(k -> (0.5f0 + t.p[k][1]/(2Rw), 0.5f0 + t.p[k][3]/(2Rw)), 3)
            else
                tex = tytread
                uv = t.uv
                if !hasuv
                    a1 = atan(t.p[1][3], t.p[1][1]); a2 = atan(t.p[2][3], t.p[2][1]); a3 = atan(t.p[3][3], t.p[3][1])
                    a2 - a1 >  Float32(pi) && (a2 -= 2f0*Float32(pi)); a2 - a1 < -Float32(pi) && (a2 += 2f0*Float32(pi))
                    a3 - a1 >  Float32(pi) && (a3 -= 2f0*Float32(pi)); a3 - a1 < -Float32(pi) && (a3 += 2f0*Float32(pi))
                    tv(y) = (y - y0) / (y1 - y0)
                    uv = ((Float32(a1/(2pi)*NREP), Float32(tv(t.p[1][2]))),
                          (Float32(a2/(2pi)*NREP), Float32(tv(t.p[2][2]))),
                          (Float32(a3/(2pi)*NREP), Float32(tv(t.p[3][2]))))
                end
            end

            push!(dressed, CLASSIC3DO.Tri(t.p, t.n, uv, tex, (1f0,1f0,1f0), false, t.ptype))
        end
        kept = dressed
    end
    qk(p) = (round(Int,p[1]*2000), round(Int,p[2]*2000), round(Int,p[3]*2000))

    if dedup === nothing; dedup = track; end
    if dedup !== false

        nkey(t) = begin
            e1 = (t.p[2][1]-t.p[1][1], t.p[2][2]-t.p[1][2], t.p[2][3]-t.p[1][3])
            e2 = (t.p[3][1]-t.p[1][1], t.p[3][2]-t.p[1][2], t.p[3][3]-t.p[1][3])
            n  = (e1[2]*e2[3]-e1[3]*e2[2], e1[3]*e2[1]-e1[1]*e2[3], e1[1]*e2[2]-e1[2]*e2[1])
            a  = abs.(n); i = a[1] >= a[2] ? (a[1] >= a[3] ? 1 : 3) : (a[2] >= a[3] ? 2 : 3)
            i * (n[i] < 0 ? -1 : 1)
        end
        ckey(t) = (round(Int,(t.p[1][1]+t.p[2][1]+t.p[3][1])/3*64),
                   round(Int,(t.p[1][2]+t.p[2][2]+t.p[3][2])/3*64),
                   round(Int,(t.p[1][3]+t.p[2][3]+t.p[3][3])/3*64),
                   round(Int, triarea(t.p)*1f5),
                   dedup === :orient ? nkey(t) : 0)
        sort!(kept, by = t -> (t.tex == "" ? 1 : 0))
        seen = Set{NTuple{5,Int}}()
        kept = filter(kept) do t
            k = ckey(t)
            (k in seen) ? false : (push!(seen, k); true)
        end
    end

    nsum = Dict{NTuple{3,Int32},NTuple{3,Float32}}()
    if smooth
        for t in kept, i in 1:3
            k=qk(t.p[i]); n=t.n[i]; s=get(nsum,k,(0f0,0f0,0f0))
            nsum[k]=(s[1]+n[1], s[2]+n[2], s[3]+n[3])
        end
    end

    crease = track ? Float32(cosd(parse(Float64, get(ENV, "JM_TRACK_CREASE", "60")))) : -2f0
    nlist = Dict{NTuple{3,Int32},Vector{NTuple{3,Float32}}}()
    if smooth && crease > -1
        for t in kept, i in 1:3; push!(get!(() -> NTuple{3,Float32}[], nlist, qk(t.p[i])), t.n[i]); end
    end
    sm(p,fallback) = begin
        if crease > -1
            s = (0f0,0f0,0f0)
            for n in get(nlist, qk(p), ()); (n[1]*fallback[1] + n[2]*fallback[2] + n[3]*fallback[3] >= crease) && (s = (s[1]+n[1], s[2]+n[2], s[3]+n[3])); end
        else
            s = get(nsum, qk(p), (0f0,0f0,0f0))
        end
        l = sqrt(s[1]^2+s[2]^2+s[3]^2)
        l < 1f-6 ? fallback : (s[1]/l, s[2]/l, s[3]/l)
    end
    texplanes = flatcol ? [_triplane(t) for t in kept if !t.flat && t.tex != ""] : NTuple{4,Float32}[]
    for t in kept

        if get(ENV,"JM_TRIDUMP","") != "" && get(ENV,"JM_TRIDUMP","") == t.tex
            _uveq = (t.uv[1] == t.uv[2] == t.uv[3])
            println("  [tri] tex=", t.tex, " flat=", t.flat, " uv_coincident=", _uveq,
                    " ptype=0x", string(t.ptype, base=16), " col=", round.(t.col, digits=3),
                    " uv1=", round.(t.uv[1], digits=3))
        end

        liv_flat = t.flat && t.tex != "" && t.tex == livery

        isvisor = visor && VISOR_ON && liv_flat && abs(t.col[1] - visorcol[1]) < 0.04f0 && abs(t.col[2] - visorcol[2]) < 0.04f0 &&
                  abs(t.col[3] - visorcol[3]) < 0.04f0
        FLATPOLY_GLASS && liv_flat && continue
        flatpoly = !isvisor && FLATPOLY_FIX && !FLATPOLY_PLANAR && liv_flat
        planar   = !isvisor && FLATPOLY_PLANAR && liv_flat

        skinned = skin !== nothing && t.tex == "" && !t.flat &&
                  (cx = (t.p[1][1] + t.p[2][1] + t.p[3][1])/3; cy = (t.p[1][2] + t.p[2][2] + t.p[3][2])/3; cz = (t.p[1][3] + t.p[2][3] + t.p[3][3])/3;
                   skin[2] < cx < skin[3] && cz > skin[4] && abs(cy) < skin[5])

        objflat = flatcol && t.flat && !isvisor && !skinned && !_inplane(t, texplanes)

        v = get!(groups, skinned ? skin[1] : isvisor ? "__visor__" : objflat ? "__flat__" : (flatpoly ? "" : t.tex), Float32[])
        mz = mirror ? -1f0 : 1f0
        for i in 1:3
            p=t.p[i]; n = smooth ? sm(p, t.n[i]) : t.n[i]; uv=t.uv[i]

            planar && (uv = (PLANAR_SIDE && abs(t.n[i][3]) < 0.5f0) ? PLANAR_SIDE_UV :
                            PLANAR_BAND ? (PLANAR_U0 - PLANAR_UW*p[2], 0.25f0 + 0.25f0*clamp((p[1] + 0.18f0)/1.22f0, 0f0, 1f0)) :
                            ((p[2] + PLANAR_W/2f0)/PLANAR_W, (p[1] - PLANAR_X0)/PLANAR_L))

            c = (cockpit_clean && t.tex=="" && grey[1] > 0.2f0) ? grey : (tint === nothing ? t.col : tint)

            if cockpit_clean && t.tex=="" && !(grey[1] > 0.2f0)

                if get(ENV,"JM_COWL_HARM","1") != "0"

                    zmx = max(t.p[1][3], t.p[2][3], t.p[3][3])
                    fw  = parse(Float32, get(ENV,"JM_FOOTWELL_Z","0.22"))
                    c = (c[2] > c[1]+0.04f0 && c[2] > c[3]+0.04f0 && zmx >= fw) ? (0.07f0,0.24f0,0.12f0) :
                                                                                  (0.10f0,0.105f0,0.115f0)
                else
                    c = (min(c[1],0.30f0), min(c[2],0.38f0), min(c[3],0.30f0))
                end
            end
            append!(v, (p[1],p[3],p[2]*mz, n[1],n[3],n[2]*mz, c[1],c[2],c[3], uflip ? 1f0-uv[1] : uv[1], vflip ? 1f0-uv[2] : uv[2]))
        end
    end
    [TrackPart(v, tex, grey) for (tex,v) in groups]
end

const FLATPOLY_FIX = get(ENV,"JM_FLATPOLY","planar") != "0"
const VISOR_ON = get(ENV, "JM_VISOR", "1") != "0"
const FLATPOLY_GLASS = lowercase(get(ENV,"JM_FLATPOLY","planar")) == "glass"

const FLATPOLY_PLANAR = lowercase(get(ENV,"JM_FLATPOLY","planar")) == "planar"

const PLANAR_W = parse(Float32, get(ENV,"JM_PLANAR_W","-1.32"))
const PLANAR_X0 = parse(Float32, get(ENV,"JM_PLANAR_X0","2.64"))
const PLANAR_L = parse(Float32, get(ENV,"JM_PLANAR_L","-3.24"))
const PLANAR_SIDE = get(ENV,"JM_PLANAR_SIDE","1") != "0"
const PLANAR_BAND = get(ENV,"JM_PLANAR_BAND","1") != "0"
const PLANAR_SIDE_UV = let v = parse.(Float32, split(get(ENV,"JM_PLANAR_SIDE_UV","0.3,0.42"), ",")); (v[1], v[2]) end

const PLANAR_U0 = parse(Float32, get(ENV, "JM_PLANAR_U0", "0.37"))
const PLANAR_UW = parse(Float32, get(ENV, "JM_PLANAR_UW", "0.184"))
const STEER_TEX = ("sterlot","lotster","lsterlog")
const CAR_NATIVE_UV = get(ENV, "JM_CAR_UNMIRROR", "1") != "0"
"""Extract the steering wheel as its own parts + pivot (centre, column axis) in the
rig frame (X fwd, Y up, Z left), so the app can rotate it with steering input."""
function extract_classic_steering(path3do)
    m = CLASSIC3DO.parse_3do(path3do)
    groups = Dict{String,Vector{Float32}}()
    cx=cy=cz=0.0; nx=ny=nz=0.0; n=0
    for t in m.tris
        (t.tex == "sterlot" || t.tex == "lsterlog") || continue
        v = get!(groups, t.tex, Float32[])
        for i in 1:3
            p=t.p[i]; nn=t.n[i]; uv=t.uv[i]

            badge = (t.tex == "lsterlog" && !haskey(ENV,"JM_BADGE_VFLIP"))
            uf = (badge && !CAR_NATIVE_UV) ? 1f0-uv[1] : uv[1]
            vf = (badge || CAR_NATIVE_UV)  ? uv[2]     : 1f0-uv[2]
            append!(v, (p[1],p[3],p[2], nn[1],nn[3],nn[2], 0.7f0,0.72f0,0.74f0, uf, vf))
            cx+=p[1]; cy+=p[3]; cz+=p[2]; nx+=nn[1]; ny+=nn[3]; nz+=nn[2]; n+=1
        end
    end
    n==0 && return (TrackPart[], Float32[0,0,0], Float32[1,0,0])
    center = Float32[cx/n, cy/n, cz/n]
    al = normalize([nx,ny,nz]); axis = Float32[al[1],al[2],al[3]]

    if haskey(groups, "lsterlog")
        v = groups["lsterlog"]; ε = 0.003f0
        for i in 0:11:length(v)-11
            v[i+1] += ε*axis[1]; v[i+2] += ε*axis[2]; v[i+3] += ε*axis[3]
        end
    end
    ([TrackPart(v, tex, (0.7f0,0.72f0,0.74f0)) for (tex,v) in groups], center, axis)
end

"""Build textured Items for Classic parts, resolving each texture via the .mip index."""

const _TEXSTAT = (uploads=Ref(0), lookups=Ref(0), names=Set{String}(),
                  tdec=Ref(0.0), tgl=Ref(0.0))

const _TEXCACHE = Dict{String,GLuint}()

const UVRANGE = Dict{GLuint,NTuple{4,Float32}}()
function uvrange_note!(tid, v)
    umn, umx, vmn, vmx = get(UVRANGE, tid, (Inf32, -Inf32, Inf32, -Inf32))
    @inbounds for i in 0:(length(v) ÷ 11 - 1)
        u = v[11i + 10]; w = v[11i + 11]
        umn = min(umn, u); umx = max(umx, u); vmn = min(vmn, w); vmx = max(vmx, w)
    end
    UVRANGE[tid] = (umn, umx, vmn, vmx)
    nothing
end
"Set GL_CLAMP_TO_EDGE on each axis a texture never tiles (call once, after every build_classic). Returns (S, T) clamped counts."
function clamp_untiled_textures!()
    get(ENV, "JM_TEX_EDGECLAMP", "1") == "0" && return (0, 0)
    ns = 0; nt = 0; e = 0.002f0
    for (tid, (umn, umx, vmn, vmx)) in UVRANGE
        glBindTexture(GL_TEXTURE_2D, tid)
        (umn >= -e && umx <= 1 + e) && (glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); ns += 1)
        (vmn >= -e && vmx <= 1 + e) && (glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); nt += 1)
    end
    glBindTexture(GL_TEXTURE_2D, 0)
    (ns, nt)
end
"""E102 S16: split a part whose triangles fall into two fore/aft clusters separated by a large gap.

`fsusp:1` measured 102 vertices at x <= -0.94 and 180 at x >= 1.35 with 2.29 m of nothing between
them (S15) -- one draw call holding two groups two metres apart, which is a MERGE, and the rear group
gets the front suspension's model matrix. Splitting is the precondition for any fix: it puts the
stray group in its own item so it can be tinted, re-placed or dropped on its own.

Splits by TRIANGLE (never through one), at the midpoint of the largest empty gap, and only when that
gap exceeds `gap` metres. JM_SPLIT_CLUSTERS=<metres> enables it; unset = off, nothing changes."""
function split_x_clusters(parts, gap::Float64)
    out = TrackPart[]
    for p in parts
        v = p.verts; nv = length(v) ÷ 11; ntri = nv ÷ 3
        if ntri < 2
            push!(out, p); continue
        end
        tx = Vector{Float32}(undef, ntri)
        for t in 0:(ntri-1)
            tx[t+1] = (v[11*(3t)+1] + v[11*(3t+1)+1] + v[11*(3t+2)+1]) / 3f0
        end
        ord = sortperm(tx)
        bestgap = 0f0; bestat = 0f0
        for k in 1:(ntri-1)
            g = tx[ord[k+1]] - tx[ord[k]]
            if g > bestgap; bestgap = g; bestat = (tx[ord[k+1]] + tx[ord[k]]) / 2f0; end
        end
        if bestgap <= Float32(gap)
            push!(out, p); continue
        end
        lo = Float32[]; hi = Float32[]
        for t in 0:(ntri-1)
            dst = tx[t+1] < bestat ? lo : hi
            append!(dst, @view v[(33t+1):(33t+33)])
        end
        isempty(lo) || push!(out, TrackPart(lo, p.tex, p.col))
        isempty(hi) || push!(out, TrackPart(hi, p.tex, p.col))
        println("  [split] ", p.tex == "" ? "(untextured)" : p.tex, " ", ntri, " tris -> ",
                length(lo) ÷ 33, " + ", length(hi) ÷ 33, " at x=", round(bestat, digits=2),
                " (gap ", round(bestgap, digits=2), " m)")
        flush(stdout)
    end
    out
end

function build_classic(parts0, idx::CLASSICTex; tag::String="")

    parts = let e = get(ENV, "JM_SPLIT_CLUSTERS", "")
        (e == "" || tag == "") ? parts0 :
            split_x_clusters(parts0, something(tryparse(Float64, e), 1.0))
    end
    cache=_TEXCACHE; items=Item[]

    _tint = get(ENV, "JM_TINT_ITEM", "")
    _tintlo, _tinthi = 0, 0
    if _tint != "" && tag != ""
        pr = split(_tint, ":")
        if length(pr) == 2 && lowercase(strip(pr[1])) == lowercase(tag)
            spec = strip(pr[2])
            if occursin('-', spec)
                ab = split(spec, '-')
                _tintlo = something(tryparse(Int, strip(ab[1])), 0)
                _tinthi = something(tryparse(Int, strip(ab[2])), 0)
            else
                _tintlo = something(tryparse(Int, spec), 0)
                _tinthi = _tintlo
            end
        end
    end
    _intint(k) = _tintlo > 0 && _tintlo <= k <= _tinthi
    for p in parts
        if _intint(length(items) + 1)
            v = p.verts; nv = length(v) ÷ 11

            for i in 0:(nv-1)
                v[11i+7] = 1.0f0; v[11i+8] = 0.0f0; v[11i+9] = 1.0f0
            end
            println("  [tint-item] ", tag, ":", length(items) + 1, " -> magenta on ", nv,
                    " vertices (tex=\"", p.tex, "\")")
            flush(stdout)
        end
        vao,n = upload(p.verts); tid=GLuint(0)
        if p.tex != "" && p.tex != "__flat__"
            key=lowercase(p.tex)
            _TEXSTAT.lookups[] += 1; push!(_TEXSTAT.names, key)
            tid = get!(cache,key) do

                _TEXSTAT.uploads[] += 1
                _t0 = time(); r = tex_rgba(idx, key); _TEXSTAT.tdec[] += time() - _t0

                if get(ENV,"JM_TEXDUMP","") == key && r !== nothing
                    let (w_, h_, px_) = (r[1], r[2], r[3])
                        open(joinpath(get(ENV, "JM_TEXDUMP_DIR", "/tmp"), "tex_" * key * ".ppm"), "w") do io
                            write(io, "P6\n$(w_) $(h_)\n255\n")
                            for i in 0:(w_*h_-1)
                                write(io, px_[4i+1], px_[4i+2], px_[4i+3])
                            end
                        end
                        println("  [texdump] ", key, " ", w_, "x", h_, " -> ", joinpath(get(ENV, "JM_TEXDUMP_DIR", "/tmp"), "tex_" * key * ".ppm"))
                    end
                end
                _t1 = time()
                t = r === nothing ? GLuint(0) :
                    (is_groove_tex(key) && get(ENV, "JM_GROOVE_SOFT", "1") != "0") ?
                        (tg = upload_rgba(r[1], r[2], soften_groove(r[1], r[2], r[3])); CUTOUT_TEX[tg] = false; tg) :
                        upload_rgba(r[1], r[2], r[3])
                _TEXSTAT.tgl[] += time() - _t1
                is_crowd_texname(key) && mag_nearest(t)
                t
            end
        end

        push!(items, Item(vao, n, _intint(length(items) + 1) ? GLuint(0) : tid, p.col))
        tid != 0 && uvrange_note!(tid, p.verts)

        if get(ENV,"JM_ITEMDUMP","") != ""
            v = p.verts; nv = length(v) ÷ 11
            xs = (typemax(Float32), typemin(Float32)); ys = xs; zs = xs

            nrear = 0; nfront = 0; rearmax = typemin(Float32); frontmin = typemax(Float32)
            for i in 0:(nv-1)
                x = v[11i+1]; y = v[11i+2]; z = v[11i+3]
                xs = (min(xs[1],x), max(xs[2],x))
                ys = (min(ys[1],y), max(ys[2],y))
                zs = (min(zs[1],z), max(zs[2],z))
                if x < 0f0
                    nrear += 1; rearmax = max(rearmax, x)
                else
                    nfront += 1; frontmin = min(frontmin, x)
                end
            end
            println("  [item] ", (tag == "" ? "?" : tag), ":", lpad(length(items),3), "  tex=\"", p.tex, "\"  tris=", n ÷ 3,
                    "  bbox x[", round(xs[1],digits=2), ",", round(xs[2],digits=2),
                    "] y[", round(ys[1],digits=2), ",", round(ys[2],digits=2),
                    "] z[", round(zs[1],digits=2), ",", round(zs[2],digits=2), "]",
                    "  x<0:", nrear, " x>=0:", nfront,
                    (nrear > 0 && nfront > 0) ?
                        string("  gap[", round(rearmax,digits=2), ",", round(frontmin,digits=2), "]=",
                               round(frontmin - rearmax, digits=2)) : "")
        end
    end
    if get(ENV,"JM_TEXSTAT","0") != "0" && _TEXSTAT.uploads[] > 0 && (_TEXSTAT.uploads[] % 100) == 0
        println("[texstat] uploads=", _TEXSTAT.uploads[], "  distinct=", length(_TEXSTAT.names),
                "  redundant=", _TEXSTAT.uploads[] - length(_TEXSTAT.names),
                "  | decode ", round(_TEXSTAT.tdec[], digits=2), " s (",
                round(1000*_TEXSTAT.tdec[]/max(_TEXSTAT.uploads[],1), digits=1), " ms ea)",
                "  GL upload ", round(_TEXSTAT.tgl[], digits=2), " s (",
                round(1000*_TEXSTAT.tgl[]/max(_TEXSTAT.uploads[],1), digits=1), " ms ea)")
        flush(stdout)
    end
    items
end

"""Read a Classic billboard stub .3do → (height, width, texture-name candidates).
The SZYX vertices give the sprite size: z-extent = height, horizontal extent = width.
Some stubs carry a real quad (e.g. single1 = 2×3m), some just a height marker
(tree1 z=7), some nothing (flagger = one (0,0,0) vertex → human-scale default).
width=0 means "derive from the texture aspect"."""
function billboard_stub(path)
    b = read(path)
    u32(o)=(o<0||o+4>length(b)) ? UInt32(0) : UInt32(b[o+1])|(UInt32(b[o+2])<<8)|(UInt32(b[o+3])<<16)|(UInt32(b[o+4])<<24)
    f32(o)=reinterpret(Float32,u32(o)); tg(o)=String(b[o+1:o+4])
    xyz=strn=0; nv=0; strnsz=0; prim=0; primsz=0; o=12
    while o+12<=length(b)
        t=tg(o); sz=Int(u32(o+8)); d=o+12
        t=="SZYX" && (xyz=d; nv=sz÷16); t=="NRTS" && (strn=d; strnsz=sz); t=="MIRP" && (prim=d; primsz=sz)
        o=d+sz; o+=(4-o%4)%4
    end
    zmn=Inf32; zmx=-Inf32; xmn=Inf32; xmx=-Inf32; ymn=Inf32; ymx=-Inf32
    for k in 0:nv-1
        x=f32(xyz+k*16+4); y=f32(xyz+k*16+8); z=f32(xyz+k*16+12)
        zmn=min(zmn,z); zmx=max(zmx,z); xmn=min(xmn,x); xmx=max(xmx,x); ymn=min(ymn,y); ymx=max(ymx,y)
    end
    zext = nv>0 ? zmx-zmn : 0f0
    hext = nv>1 ? max(xmx-xmn, ymx-ymn) : 0f0

    sprh = 0f0
    if zext <= 0.5f0 && prim > 0
        for q in prim:4:prim+primsz-24

            if u32(q) == 3 && (u32(q+8) == 0xFFFFFFFF || (u32(q+4) == 0 && u32(q+12) == 0 && u32(q+20) == 65536))
                hh = f32(q+16); (isfinite(hh) && 0.5f0 < hh < 200f0) && (sprh = hh; break)
            end
        end
    end
    height = zext > 0.5f0 ? zext : sprh > 0f0 ? sprh : 2.5f0
    width  = hext > 0.5f0 ? hext : 0f0

    aax = 0f0
    if nv > 1 && hext > 0.5f0
        best = 0f0; bi = 0; bj = 0
        for k in 0:nv-1, l in k+1:nv-1
            dx = f32(xyz+l*16+4) - f32(xyz+k*16+4); dy = f32(xyz+l*16+8) - f32(xyz+k*16+8)
            d2 = dx*dx + dy*dy
            d2 > best && (best = d2; bi = k; bj = l)
        end
        aax = Float32(atan(f32(xyz+bj*16+8) - f32(xyz+bi*16+8), f32(xyz+bj*16+4) - f32(xyz+bi*16+4)))
    end
    strs=String[]; cur=UInt8[]
    for i in strn:strn+strnsz-1
        c=b[i+1]; c==0xFF && break
        c==0x00 ? (push!(strs,String(copy(cur))); empty!(cur)) : push!(cur,c)
    end
    !isempty(cur) && push!(strs,String(copy(cur)))
    (height, width, strs, aax)
end

"""Build a camera-facing billboard Item for a texture name → (Item, texW, texH) or
nothing.  Unit quad: x∈[-0.5,0.5], y∈[0,1] (stands on the ground), z=0.  Normal points
UP so the sprite is lit bright (not darkened by a facing-the-camera Lambert term)."""

const ANISO = parse(Float32, get(ENV,"JM_ANISO","16"))
const GL_TEXTURE_MAX_ANISOTROPY_EXT = 0x84FE
const ANISO_REPORTED = Ref(false)
function set_anisotropy()
    ANISO > 1f0 || return

    err = false
    try
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, ANISO)
    catch e
        err = true
        ANISO_REPORTED[] || println("  [aniso] glTexParameterf THREW: ", typeof(e))
    end
    if !ANISO_REPORTED[]
        ANISO_REPORTED[] = true
        got = Float32[0f0]; mx = Float32[0f0]
        try
            glGetTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, got)
            glGetFloatv(0x84FF, mx)
        catch
        end
        println("  [aniso] requested ", ANISO, " → texture reports ", got[1],
                "   (driver maximum ", mx[1], ")", err ? "  [SET FAILED]" : "")
        flush(stdout)
    end
end

function build_billboard(texname, idx::CLASSICTex)
    r = tex_rgba(idx, texname); r === nothing && return nothing
    tid = upload_rgba(r[1], r[2], r[3])
    q=Float32[]
    vtx(x,y,u,v) = append!(q, (x,y,0f0, 0f0,1f0,0f0, 1f0,1f0,1f0, u,v))
    vtx(-0.5f0,0f0,0f0,1f0); vtx(0.5f0,0f0,1f0,1f0); vtx(0.5f0,1f0,1f0,0f0)
    vtx(-0.5f0,0f0,0f0,1f0); vtx(0.5f0,1f0,1f0,0f0); vtx(-0.5f0,1f0,0f0,0f0)
    vao,n = upload(q)
    (Item(vao,n,tid,(1f0,1f0,1f0)), Float32(r[1]), Float32(r[2]))
end

"""Build the 12-panel Classic horizon ring as backdrop Items (one textured quad each).
`R` ring radius (kept < far plane); `e_lo`/`e_hi` bottom/top elevation; the
silhouette band (bottom ~20% of each panel) straddles e_lo→0 so it sits on the
horizon line.  `yaw0` rotates the whole ring to register it with the track."""
function build_horizon(idx::CLASSICTex; R=2500f0, e_lo=deg2rad(-6f0), e_hi=deg2rad(24f0), yaw0=0f0, n=12)
    items = Item[]

    if tex_rgba(idx, "horiz0") !== nothing && tex_rgba(idx, "horiz1") === nothing
        r = tex_rgba(idx, "horiz0"); tid = upload_rgba(r[1], r[2], r[3])

        ns = 48; dθ = 2f0π/ns
        slo = deg2rad(parse(Float32, get(ENV,"JM_STRIP_LO","-5")))
        shi = deg2rad(parse(Float32, get(ENV,"JM_STRIP_HI","2.2")))
        vtop = parse(Float32, get(ENV,"JM_STRIP_VTOP","0.52"))
        h_lo = R*tan(Float32(slo)); h_hi = R*tan(Float32(shi))
        q=Float32[]
        sv(x,y,z,u,v)=append!(q,(x,y,z, 0f0,0f0,0f0, 1f0,1f0,1f0, u,v))
        for i in 0:ns-1
            θ0=Float32(yaw0+i*dθ); θ1=Float32(yaw0+(i+1)*dθ)
            x0=R*cos(θ0); z0=R*sin(θ0); x1=R*cos(θ1); z1=R*sin(θ1)
            u0=Float32(i/ns); u1=Float32((i+1)/ns)
            sv(x0,h_lo,z0, u0,1f0); sv(x1,h_lo,z1, u1,1f0); sv(x1,h_hi,z1, u1,vtop)
            sv(x0,h_lo,z0, u0,1f0); sv(x1,h_hi,z1, u1,vtop); sv(x0,h_hi,z0, u0,vtop)
        end
        vao,m = upload(q)
        push!(items, Item(vao,m,tid,(1f0,1f0,1f0)))
        return items
    end

    bandhi = deg2rad(parse(Float32, get(ENV,"JM_RING_HI","6"))); vtop = parse(Float32, get(ENV,"JM_RING_VTOP","0.40"))
    h_lo = R*tan(Float32(e_lo)); h_hi = R*tan(Float32(bandhi)); dθ = 2f0π/n
    for i in 0:n-1
        r = tex_rgba(idx, "horiz$(i)"); r === nothing && continue
        tid = upload_rgba(r[1], r[2], r[3])
        θ0 = Float32(yaw0 + i*dθ); θ1 = Float32(yaw0 + (i+1)*dθ)
        x0=R*cos(θ0); z0=R*sin(θ0); x1=R*cos(θ1); z1=R*sin(θ1)
        q=Float32[]
        vtx(x,y,z,u,v)=append!(q,(x,y,z, 0f0,0f0,0f0, 1f0,1f0,1f0, u,v))
        vtx(x0,h_lo,z0, 0f0,1f0); vtx(x1,h_lo,z1, 1f0,1f0); vtx(x1,h_hi,z1, 1f0,vtop)
        vtx(x0,h_lo,z0, 0f0,1f0); vtx(x1,h_hi,z1, 1f0,vtop); vtx(x0,h_hi,z0, 0f0,vtop)
        vao,m = upload(q)
        push!(items, Item(vao,m,tid,(1f0,1f0,1f0)))
    end
    items
end

const HORIZ_KEY = Ref(false)
const HORIZ_K = Ref(0f0)
"""HORIZ3DO-1 (2026-09-25, gold parity): Classic's OWN horizon -- the track's `horiz.3do` -- as a camera-
centred backdrop. build_horizon above guesses a layout from panel names, and its texture lookup
prefix-matches: at Watkins Glen `horiz0` matched ONE of the four 90-degree panels horiz01..04 and wrapped it
round 360 degrees (a 4x-stretched, blurred smear where the gold shows crisp hills). horiz.3do places every
panel exactly (WG: hills at elevation 0..4.29 deg over a `shoriz` band -4.29..0; Zandvoort: six panels +
thirteen cloud cards; the Ring: its own sky and ground). Untextured triangles (the 1e9 m sky box) are
skipped; the rest is scaled uniformly to radius R so it sits inside the far plane with every angle kept.
`path` = a .3do on disk. Returns Item[] (empty if the file has no textured panels)."""
function build_horizon_3do(idx::CLASSICTex, path::AbstractString; R=2500f0)
    m = CLASSIC3DO.parse_3do(path)
    tris = [t for t in m.tris if t.tex != "" && all(hypot(q[1], q[2]) < 1f6 for q in t.p)]
    isempty(tris) && return Item[]
    HORIZ_KEY[] = !any(t -> lowercase(t.tex) == "sky", tris)
    rmax = maximum(hypot(q[1], q[2], q[3]) for t in tris for q in t.p)
    k = Float32(R / rmax)
    HORIZ_K[] = k
    by = Dict{String,Vector{Float32}}()
    for t in tris, i in 1:3
        p = t.p[i]; uv = t.uv[i]
        append!(get!(by, t.tex, Float32[]), (k*p[1], k*p[3], -k*p[2], 0f0, 1f0, 0f0, 1f0, 1f0, 1f0, uv[1], uv[2]))
    end
    items = Item[]
    for (tex, q) in by
        r = tex_rgba(idx, tex); r === nothing && continue
        tid = upload_rgba(r[1], r[2], r[3]); vao, n = upload(q)
        push!(items, Item(vao, n, tid, (1f0, 1f0, 1f0)))
    end
    items
end

"""Draw the horizon ring centred on the camera, unlit + unfogged, behind the scene
(depth-write off so closer geometry overwrites it)."""
function draw_horizon(prog, ring, vp, campos; tint=(1f0,1f0,1f0))
    isempty(ring) && return
    glUniform1i(uloc(prog,"uSky"), 1)
    u3(prog,"uSkyTint",tint)
    glUniform1i(uloc(prog,"uSkyKey"), HORIZ_KEY[] ? 1 : 0)
    glDepthMask(GL_FALSE)

    M = translate(Float32[campos[1], (1f0 - HORIZ_K[])*campos[2], campos[3]])
    for it in ring; draw(prog, it, vp, M; bright=1.0); end
    glDepthMask(GL_TRUE)
    glUniform1i(uloc(prog,"uSky"), 0)
end

"""Model matrix for a billboard at render `pos` (base), sized `w`×`h`, yawed to face
the camera at `eye`."""

const _BBM = Matrix{Float32}(I, 4, 4)
function billboard_model(pos, w, h, eye)
    yaw = atan(eye[1]-pos[1], eye[3]-pos[3]); c = Float32(cos(yaw)); sn = Float32(sin(yaw))
    M = _BBM
    M[1,1] = c*Float32(w); M[1,2] = 0f0;         M[1,3] = sn;  M[1,4] = Float32(pos[1])
    M[2,1] = 0f0;          M[2,2] = Float32(h);  M[2,3] = 0f0; M[2,4] = Float32(pos[2])
    M[3,1] = -sn*Float32(w); M[3,2] = 0f0;       M[3,3] = c;   M[3,4] = Float32(pos[3])
    M[4,1] = 0f0; M[4,2] = 0f0; M[4,3] = 0f0; M[4,4] = 1f0
    M
end

struct Item; vao::GLuint; n::GLsizei; tex::GLuint; col::NTuple{3,Float32}; end

"""Axis-aligned bbox over the rig-frame positions of TrackParts (verts stride 11,
pos at offsets 1:3).  Returns a NamedTuple (xmin,xmax,ymin,ymax,zmin,zmax)."""
function parts_bbox(parts)
    xmn=ymn=zmn=Inf32; xmx=ymx=zmx=-Inf32
    for p in parts
        v = p.verts
        @inbounds for i in 1:11:length(v)-10
            x=v[i]; y=v[i+1]; z=v[i+2]
            xmn=min(xmn,x); xmx=max(xmx,x); ymn=min(ymn,y); ymx=max(ymx,y); zmn=min(zmn,z); zmx=max(zmx,z)
        end
    end
    (xmin=xmn, xmax=xmx, ymin=ymn, ymax=ymx, zmin=zmn, zmax=zmx)
end

"A loaded Classic car: body Items, a name→Items wheel cache, an auto-derived body
offset (rig frame), and the wheel placement spec (hubX, hubZ, steers?, radius, mesh)."
struct CLASSICCarModel
    name::String
    body::Vector{Item}
    wheels::Dict{String,Vector{Item}}
    body_off::NTuple{3,Float32}
    wheelspec::Vector{Tuple{Float32,Float32,Bool,Float32,String}}
end

"""Load a Classic '67 car (body + wheels) from its `cars67/<dir>` folder, AUTO-LEVELLED
to sit centred on its wheels.  `body_floor` = the world-Y the body underside should
reach (pass the player Lotus's `bbox.ymin + BODY_OFF[2]` so the whole grid sits at a
common height).  `wheelspec` reuses the Lotus hub geometry (all '67 cars are
dimensionally near-identical) with each car's own wheel mesh names; a missing mesh
loads as nothing (drawn wheel-less rather than crashing)."""

const _WRAPPER_CACHE = Dict{String,Dict{String,NTuple{3,String}}}()
const _WRAPPER_PATHS = Dict{String,String}()
function wheel_dress_for(dir, meshname)
    tbl = get!(_WRAPPER_CACHE, dir) do
        d = Dict{String,NTuple{3,String}}()
        for f in readdir(dir; join=true)
            endswith(lowercase(f), ".3do") || continue
            filesize(f) > 512 && continue
            b = try read(f) catch; continue end

            (length(b) > 24 && String(b[1:4]) == "4OD3") || continue

            o = 12; names = String[]
            while o + 12 <= length(b)
                t = String(b[o+1:o+4]); sz = Int(reinterpret(UInt32, b[o+9:o+12])[1]); data = o + 12
                if t == "NRTS"
                    cur = UInt8[]
                    for i in data+1:min(data+sz, length(b))
                        c = b[i]
                        c == 0xFF && break
                        c == 0x00 ? (push!(names, String(copy(cur))); empty!(cur)) : push!(cur, c)
                    end
                    break
                end
                o = data + sz; o += (4 - o % 4) % 4
            end
            length(names) >= 4 || continue

            if !haskey(d, lowercase(names[1]))
                d[lowercase(names[1])] = (names[2], names[3], names[4])
                _WRAPPER_PATHS[lowercase(names[1])] = f
            end
        end
        d
    end
    get(tbl, lowercase(meshname), nothing)
end

function mesh_wheel_hubs(path3do)
    m = CLASSIC3DO.parse_3do(path3do)
    acc = Dict{Tuple{String,Int},Vector{NTuple{3,Float32}}}()
    for t in m.tris, p in t.p
        push!(get!(acc, (t.tex, p[2] >= 0 ? 1 : -1), NTuple{3,Float32}[]), p)
    end
    hubs = NTuple{7,Float64}[]
    for ((tex, side), ps) in acc
        length(ps) < 30 && continue
        xl, xh = extrema(p[1] for p in ps); zl, zh = extrema(p[3] for p in ps); yl, yh = extrema(p[2] for p in ps)
        ez = zh - zl; ey = yh - yl
        cy = sum(p[2] for p in ps)/length(ps)
        (0.5 <= ez <= 0.8 && ey <= 0.45 && abs(cy) >= 0.45) || continue
        cx = sum(p[1] for p in ps)/length(ps)
        groups = (xh - xl) <= 0.8 ? [ps] : [[p for p in ps if p[1] >= cx], [p for p in ps if p[1] < cx]]
        for q in groups
            length(q) < 15 && continue
            qxl, qxh = extrema(p[1] for p in q); (qxh - qxl) <= 0.8 || continue
            qzl, qzh = extrema(p[3] for p in q); qyl, qyh = extrema(abs(p[2]) for p in q)
            push!(hubs, (sum(p[1] for p in q)/length(q), sum(p[2] for p in q)/length(q), sum(p[3] for p in q)/length(q), (qzh - qzl)/2, length(q), qyl, qyh))
        end
    end
    length(hubs) < 4 && return nothing
    xm = sum(h[1]*h[5] for h in hubs)/sum(h[5] for h in hubs)
    front = [h for h in hubs if h[1] >= xm]; rear = [h for h in hubs if h[1] < xm]
    (isempty(front) || isempty(rear)) && return nothing
    wmean(v, i) = sum(h[i]*h[5] for h in v)/sum(h[5] for h in v)

    amid(v) = (minimum(h[6] for h in v) + maximum(h[7] for h in v)) / 2
    (fx = wmean(front, 1), rx = wmean(rear, 1), fy = amid(front), ry = amid(rear),
     fr = wmean(front, 4), rr = wmean(rear, 4), fz = wmean(front, 3), rz = wmean(rear, 3))
end

function wheelspec_from_mesh(hubs, wheelspec, off_x, off_z)
    map(wheelspec) do (bx, bz, front, r, mesh)
        x = (front ? hubs.fx : hubs.rx) + off_x
        z = sign(bz) * (front ? hubs.fy : hubs.ry) + off_z
        (Float32(x), Float32(z), front, r, mesh)
    end
end

"""FIELD7-1: `dir/name` as it is on disk, matched without regard to case -- Classic is a Windows game and its names are
case-blind (the Honda ships MURASAMA.3DO and HONDLF.3DO; an exact-case join missed them). The exact path if nothing matches."""
function _ci_path(dir, name)
    p = joinpath(dir, name); isfile(p) && return p
    isdir(dir) || return p
    ln = lowercase(name)
    for f in readdir(dir)
        lowercase(f) == ln && return joinpath(dir, f)
    end
    p
end

function load_classic_car(name, dir, body3do, wheelspec;
                      exclude=("ltraymap","lshad"), maxlat=Inf32, exclude_groups=(),
                      body_floor=0.0f0, wheeltint=(0.12f0,0.12f0,0.13f0),
                      rear_groups=(), rear_lat=0.66f0, rear_ymin=-0.12f0, hub3do=nothing, skip_ptypes=nothing,
                      sleeves=nothing, skin=nothing, drop_unbound::Bool=false)
    tex   = classic_texture_index(dir)

    wheeltex = String[]
    if get(ENV,"JM_BODY_WHEELTEX","1") != "0"
        for (_,_,_,_,mesh) in wheelspec
            wheel_dress_for(dir, mesh)

            wp = get(_WRAPPER_PATHS, lowercase(mesh), "")
            src = wp != "" ? wp : _ci_path(dir, mesh * ".3do")
            isfile(src) || continue
            for q in extract_classic_car(src; exclude=("ltraymap","lshad"))
                q.tex != "" && !(q.tex in wheeltex) && push!(wheeltex, q.tex)
            end
        end
    end
    parts = extract_classic_car(_ci_path(dir, body3do); exclude=(exclude..., wheeltex...), maxlat=maxlat, skip_ptypes_kw=skip_ptypes, drop_unbound=drop_unbound,
                            exclude_groups=(exclude_groups..., rear_groups...), drop_green=true, skin=skin)

    for g in rear_groups
        rp = extract_classic_car(_ci_path(dir, body3do); exclude=(exclude..., wheeltex...), include_groups=(g,),
                             maxlat=rear_lat, trim=true, drop_green=true)
        for p in rp
            v = p.verts; keep = Float32[]
            for t in 0:(length(v) ÷ 33)-1
                b = t*33
                (v[b+2] < rear_ymin || v[b+13] < rear_ymin || v[b+24] < rear_ymin) && continue
                append!(keep, @view v[b+1:b+33])
            end
            isempty(keep) || push!(parts, TrackPart(keep, p.tex, p.col))
        end
    end
    bb    = parts_bbox(parts)

    if sleeves !== nothing
        arm_names, hand_names, sh = sleeves
        for p in extract_classic_car(_ci_path(dir, body3do); only=hand_names, maxlat=maxlat, skip_ptypes_kw=skip_ptypes,
                                 exclude_groups=(exclude_groups..., rear_groups...))
            push!(parts, p)
        end
        arms = extract_classic_car(_ci_path(dir, body3do); only=arm_names, maxlat=maxlat, skip_ptypes_kw=skip_ptypes,
                               exclude_groups=(exclude_groups..., rear_groups...))
        for side in (-1, 1)
            ends = sleeve_axis_ends(arms, side)
            ends === nothing && continue
            W, E = ends
            S = (W[1] + sh[1], W[2] + sh[2], side * sh[3])
            get(ENV, "JM_AI_ARMS_DIAG", "0") != "0" && println("    [ai arms] ", name, " side ", side, " wrist ", round.(W, digits=2),
                " far ", round.(E, digits=2), " shoulder ", round.(S, digits=2), " arm parts ", length(arms), " (", join(unique(p.tex for p in arms), ","), ")")
            for p in arms
                v = p.verts; keep = Float32[]
                for t in 0:(length(v) ÷ 33)-1
                    b = t*33
                    sign(v[b+3] + v[b+14] + v[b+25]) == side || continue
                    append!(keep, @view v[b+1:b+33])
                end
                isempty(keep) && continue
                push!(parts, TrackPart(lay_along!(keep, W, E, W, S, sh[4]), p.tex, p.col))
            end
        end
    end
    body  = build_classic(parts, tex)
    off_x = -(bb.xmin + bb.xmax) / 2f0
    off_z = -(bb.zmin + bb.zmax) / 2f0
    off_y = body_floor - bb.ymin
    wheels = Dict{String,Vector{Item}}()
    for (_,_,_,_,mesh) in wheelspec
        haskey(wheels, mesh) && continue
        path = _ci_path(dir, mesh*".3do")

        dress = wheel_dress_for(dir, mesh)
        wpath = get(_WRAPPER_PATHS, lowercase(mesh), "")
        wheels[mesh] = wpath != "" ?
            build_classic(extract_classic_car(wpath; exclude=("ltraymap","lshad"), tint=wheeltint), tex) :
            (isfile(path) ?
             build_classic(extract_classic_car(path; exclude=("ltraymap","lshad"), tint=wheeltint,
                                       wheel_dress=dress), tex) : Item[])
    end

    if get(ENV, "JM_WHEELS_TABLE", "0") == "0"
        hubs = try mesh_wheel_hubs(_ci_path(dir, hub3do === nothing ? body3do : hub3do)) catch e; @warn "mesh_wheel_hubs failed" name e; nothing end
        if hubs !== nothing
            old = wheelspec
            wheelspec = wheelspec_from_mesh(hubs, wheelspec, off_x, off_z)
            println("  [wheels] ", name, ": hubs from the mesh  front x=", round(hubs.fx, digits=2), " rear x=", round(hubs.rx, digits=2),
                    " half-track ", round(hubs.fy, digits=2), "/", round(hubs.ry, digits=2), " (+body off ", round(off_x, digits=2), ")  was ",
                    join(["($(round(w[1],digits=2)),$(round(w[2],digits=2)))" for w in old], " "), "  now ",
                    join(["($(round(w[1],digits=2)),$(round(w[2],digits=2)))" for w in wheelspec], " "))
        else
            println("  [wheels] ", name, ": mesh tyres not found -- keeping the table")
        end
    end
    CLASSICCarModel(name, body, wheels, (Float32(off_x), Float32(off_y), Float32(off_z)),
                Vector{Tuple{Float32,Float32,Bool,Float32,String}}(wheelspec))
end

"""HANDS-2: the (wrist, far end) of a sleeve strip on one side (mesh z sign): the ends of its long axis, the wrist the lower."""
function sleeve_axis_ends(parts, side)
    P = NTuple{3,Float64}[]
    for p in parts, i in 1:11:length(p.verts)-10
        v = p.verts; sign(v[i+2]) == side && push!(P, (v[i], v[i+1], v[i+2]))
    end
    length(P) < 3 && return nothing
    c = (sum(p[1] for p in P), sum(p[2] for p in P), sum(p[3] for p in P)) ./ length(P)
    C = zeros(3, 3)
    for p in P, a in 1:3, b in 1:3; C[a, b] += (p[a] - c[a]) * (p[b] - c[b]); end
    ax = eigen(Symmetric(C)).vectors[:, 3]
    pr = [sum((p .- c) .* ax) for p in P]
    a = c .+ Tuple(ax .* minimum(pr)); b = c .+ Tuple(ax .* maximum(pr))
    a[2] <= b[2] ? (a, b) : (b, a)
end

"""HANDS-2: re-lay an interleaved vertex strip so its axis E->W runs S->H (stretched along, `thick` across), in place."""
function lay_along!(v, W, E, H, S, thick)
    a = collect(W .- E); la = sqrt(sum(a .^ 2)); a ./= la
    b = collect(H .- S); lb = sqrt(sum(b .^ 2)); b ./= lb
    x = [a[2]*b[3]-a[3]*b[2], a[3]*b[1]-a[1]*b[3], a[1]*b[2]-a[2]*b[1]]; cth = sum(a .* b); sx = sqrt(sum(x .^ 2))
    K = [0 -x[3] x[2]; x[3] 0 -x[1]; -x[2] x[1] 0]
    R = sx < 1e-9 ? (cth > 0 ? Matrix(1.0I, 3, 3) : -Matrix(1.0I, 3, 3)) : Matrix(1.0I, 3, 3) + K + K*K*((1 - cth)/sx^2)
    L = R * (thick .* Matrix(1.0I, 3, 3) .+ (lb/la - thick) .* (a * a'))
    Ln = transpose(inv(L))
    for i in 1:11:length(v)-10
        q = L * (Float64[v[i], v[i+1], v[i+2]] .- collect(W)) .+ collect(H)
        n = Ln * Float64[v[i+3], v[i+4], v[i+5]]; ln = sqrt(sum(n .^ 2)); ln > 0 && (n ./= ln)
        v[i] = q[1]; v[i+1] = q[2]; v[i+2] = q[3]; v[i+3] = n[1]; v[i+4] = n[2]; v[i+5] = n[3]
    end
    v
end

function upload(interleaved)
    vao=Ref{GLuint}(); glGenVertexArrays(1,vao); glBindVertexArray(vao[])
    vbo=Ref{GLuint}(); glGenBuffers(1,vbo); glBindBuffer(GL_ARRAY_BUFFER,vbo[])
    glBufferData(GL_ARRAY_BUFFER,sizeof(interleaved),interleaved,GL_STATIC_DRAW)
    st=11*4
    for (loc,off) in ((0,0),(1,3*4),(2,6*4),(3,9*4))
        glVertexAttribPointer(loc,(loc==3 ? 2 : 3),GL_FLOAT,false,st,Ptr{Cvoid}(off)); glEnableVertexAttribArray(loc)
    end
    Ref{GLuint}(vao[])[], GLsizei(length(interleaved)÷11)
end
setmat(prog,name,M)=glUniformMatrix4fv(uloc(prog,name),1,GL_FALSE,umat(M))

const EXPOSURE = Ref(parse(Float32, get(ENV,"JM_EXPOSURE","1.0")))
const WBAL = let v = get(ENV,"JM_WBAL","")
    if v == ""
        (0.955f0, 1f0, 1.05f0)
    else
        p = parse.(Float32, split(v, ","))
        length(p) == 3 ? (p[1],p[2],p[3]) : (1f0,1f0,1f0)
    end
end

function draw(prog, item::Item, vp, model; wet::Real=0.0, bright::Real=1.0, spec::Real=0.0, ambfill::Real=0.0, graze::Bool=false, alpha::Real=1.0, tint=(1f0,1f0,1f0), mirrorglass::Bool=false, unlit::Bool=false, depthbias::Bool=false, first::Integer=0, count::Integer=-1, flat::NTuple{2,Float32}=(0f0, 0f0))

    if depthbias
        glEnable(GLenum(0x8037))
        glPolygonOffset(-2.5f0, -4f0)
    end
    setmat(prog,"uVP",vp); setmat(prog,"uModel",model)
    glUniform1i(uloc(prog,"uUnlit"), unlit ? 1 : 0)
    glUniform3f(uloc(prog,"uTint"), Float32(tint[1]), Float32(tint[2]), Float32(tint[3]))
    glUniform1i(uloc(prog,"uMirrorGlass"), mirrorglass ? 1 : 0)
    glUniform1f(uloc(prog,"uBright"), Float32(bright))
    glUniform1f(uloc(prog,"uSpec"), Float32(spec))
    glUniform1f(uloc(prog,"uWet"), Float32(wet))
    glUniform1f(uloc(prog,"uAlpha"), Float32(alpha))
    glUniform1f(uloc(prog,"uAmbFill"), Float32(ambfill))
    glUniform1i(uloc(prog,"uFlatOn"), flat[1] > 0f0 ? 1 : 0)
    glUniform2f(uloc(prog,"uFlat"), flat[1], flat[2])
    glUniform3f(uloc(prog,"uWBal"), WBAL[1], WBAL[2], WBAL[3])
    glUniform1f(uloc(prog,"uExposure"), EXPOSURE[])
    glUniform1i(uloc(prog,"uGraze"), graze ? 1 : 0)
    glUniform1i(uloc(prog,"uCutout"), get(CUTOUT_TEX, item.tex, false) ? 1 : 0)
    if item.tex != 0
        glUniform1i(uloc(prog,"uHasTex"),1)
        glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D,item.tex)
        glUniform1i(uloc(prog,"uTex"),0)
    else
        glUniform1i(uloc(prog,"uHasTex"),0)
    end
    flip = negdet(model); flip && glFrontFace(GL_CW)
    glBindVertexArray(item.vao); glDrawArrays(GL_TRIANGLES, first, count < 0 ? item.n : count)
    flip && glFrontFace(GL_CCW)
    NDRAW[] += 1; NTRI[] += item.n ÷ 3
    depthbias && glDisable(GLenum(0x8037))
end
"""Upload a bare interleaved blob as an untextured Item (car, wheels)."""
function item(interleaved)
    vao,n = upload(interleaved); Item(vao,n,GLuint(0),(1f0,1f0,1f0))
end

const TEXT_VS = """
#version 330 core
layout(location=0) in vec2 p; layout(location=1) in vec2 t; layout(location=2) in vec3 c;
uniform vec2 uRes; out vec2 uv; out vec3 col;
void main(){ uv=t; col=c; gl_Position=vec4(p.x/uRes.x*2.0-1.0, 1.0-p.y/uRes.y*2.0, 0.0, 1.0); }"""
const TEXT_FS = """
#version 330 core
in vec2 uv; in vec3 col; uniform sampler2D uAtlas; out vec4 o;
void main(){ float a=texture(uAtlas,uv).r; o=vec4(col*a, a); }"""
function text_program()
    p=glCreateProgram(); glAttachShader(p,compile(TEXT_VS,GL_VERTEX_SHADER)); glAttachShader(p,compile(TEXT_FS,GL_FRAGMENT_SHADER)); glLinkProgram(p); p
end
struct Font
    tex::GLuint; W::Int; H::Int; lineh::Int; ascent::Int
    glyphs::Dict{Char,NTuple{7,Float32}}
end
"""Load `<dir>/font<px>.txt` + `.a8` (see make_font_atlas.py). Returns `nothing` if absent, so the
sim runs without text rather than dying on a missing asset."""
function load_font(dir, px=18)
    meta=joinpath(dir,"font$(px).txt"); raw=joinpath(dir,"font$(px).a8")
    (isfile(meta) && isfile(raw)) || return nothing
    lines=readlines(meta); W,H,lineh,asc = parse.(Int, split(lines[1]))
    a8=read(raw); length(a8)==W*H || return nothing
    g=Dict{Char,NTuple{7,Float32}}()
    for l in lines[2:end]
        f=split(l); code=parse(Int,f[1])
        g[Char(code)]=ntuple(i->Float32(parse(Float64,f[i+1])),7)
    end
    tex=Ref{GLuint}(); glGenTextures(1,tex); glBindTexture(GL_TEXTURE_2D,tex[])
    glPixelStorei(GL_UNPACK_ALIGNMENT,1)
    glTexImage2D(GL_TEXTURE_2D,0,GL_R8,W,H,0,GL_RED,GL_UNSIGNED_BYTE,a8)
    glPixelStorei(GL_UNPACK_ALIGNMENT,4)
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR); glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE); glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE)
    Font(tex[],W,H,lineh,asc,g)
end
"""Append the quads for `str` at pen position (x,y) = top-left of the line box; returns the pen x after it."""
function text!(v::Vector{Float32}, font::Font, x, y, str::AbstractString, col; scale=1.0)
    px=Float32(x); base=Float32(y)
    for ch in str
        g=get(font.glyphs, ch, nothing); g===nothing && (g=get(font.glyphs,'?',nothing)); g===nothing && continue
        gx,gy,gw,gh,xo,yo,adv=g
        x0=px+xo*scale; y0=base+yo*scale; x1=x0+gw*scale; y1=y0+gh*scale
        u0=gx/font.W; v0=gy/font.H; u1=(gx+gw)/font.W; v1=(gy+gh)/font.H
        for (qx,qy,qu,qv) in ((x0,y0,u0,v0),(x1,y0,u1,v0),(x1,y1,u1,v1),(x0,y0,u0,v0),(x1,y1,u1,v1),(x0,y1,u0,v1))
            push!(v, qx, qy, qu, qv, col[1], col[2], col[3])
        end
        px+=adv*scale
    end
    px
end
text_width(font::Font, str; scale=1.0) = sum((get(font.glyphs,ch,get(font.glyphs,'?',ntuple(_->0f0,7)))[7] for ch in str); init=0f0)*scale
function text_draw(prog,vao,vbo,font::Font,v,W,H)
    isempty(v) && return
    glDisable(GL_DEPTH_TEST); glEnable(GL_BLEND); glBlendFunc(GL_ONE,GL_ONE_MINUS_SRC_ALPHA)
    glUseProgram(prog); glUniform2f(uloc(prog,"uRes"),Float32(W),Float32(H))
    glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D,font.tex); glUniform1i(uloc(prog,"uAtlas"),0)
    glBindVertexArray(vao); glBindBuffer(GL_ARRAY_BUFFER,vbo)
    glBufferData(GL_ARRAY_BUFFER,sizeof(v),v,GL_DYNAMIC_DRAW)
    glVertexAttribPointer(0,2,GL_FLOAT,false,7*4,Ptr{Cvoid}(0));   glEnableVertexAttribArray(0)
    glVertexAttribPointer(1,2,GL_FLOAT,false,7*4,Ptr{Cvoid}(2*4)); glEnableVertexAttribArray(1)
    glVertexAttribPointer(2,3,GL_FLOAT,false,7*4,Ptr{Cvoid}(4*4)); glEnableVertexAttribArray(2)
    glDrawArrays(GL_TRIANGLES,0,length(v)÷7)
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA); glEnable(GL_DEPTH_TEST)
end

end
