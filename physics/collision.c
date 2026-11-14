#include "collision.h"
#include "geometry.h"
#include "vec.h"
#include "world.h"

int generate_contact_info(body*body_a,body*body_b,contact_info* info){
    if(body_a->shape.id==0&&body_b->shape.id==0){
        if(!circle_contact_circle(*(body_a->shape.c.p), *(body_b->shape.c.p), body_a->shape.c.r, body_b->shape.c.r))return -1;
        /* normal points from B to A, matching solve_contact's sign convention */
        vec2 n=VEC2_NORMALIZE(VEC2_SUB(*(body_a->shape.c.p), *(body_b->shape.c.p)));
        info->normal=n;
        info->tangent=VEC2_PERP(n);
        info->body_a = body_a;
        info->body_b = body_b;
        float dist = VEC2_LENGTH(VEC2_SUB(*(body_b->shape.c.p),*(body_a->shape.c.p)));
        /* contact point on A's surface facing B (dir is A->B) */
        vec2 dir=VEC2_MUL(n,-1.0f);
        info->collision_point=VEC2_ADD(*(body_a->shape.c.p), VEC2_MUL(dir, body_a->shape.c.r));
        info->depth = body_a->shape.c.r+body_b->shape.c.r-dist;
        return 0;
    }
    return -1;
}

void solve_contact(contact_info* info){
    // apply impulse
    vec2 n=info->normal;
    float inv_ma=1/info->body_a->mass;
    float inv_mb=1/info->body_b->mass;
    float inv_ia=1/info->body_a->inertia;
    float inv_ib=1/info->body_b->inertia;
    vec2 rA=VEC2_SUB(info->collision_point, info->body_a->position);
    vec2 rB=VEC2_SUB(info->collision_point, info->body_b->position);

    vec3 vA= VEC3_ADD((vec3){info->body_a->linear_velocity.x,info->body_a->linear_velocity.y,0.0},VEC3_CROSS(info->body_a->angular_velocity, (vec3){rA.x,rA.y,0.0}));
    vec3 vB= VEC3_ADD((vec3){info->body_b->linear_velocity.x,info->body_b->linear_velocity.y,0.0},VEC3_CROSS(info->body_b->angular_velocity, (vec3){rB.x,rB.y,0.0}));

    vec3 v_rel=VEC3_SUB(vA,vB);
    float vn = VEC2_DOT((vec2){v_rel.x,v_rel.y},n);
    if (vn>0)return;
    
    float rnA = rA.x*n.y-rA.y*n.x;
    float rnB = rB.x*n.y-rB.y*n.x;
    float inv_meff_n = (inv_ma + inv_mb + inv_ia * rnA * rnA + inv_ib * rnB * rnB);
    if (inv_meff_n==0)return;
    float jn = -(1.0 + 1.0) * vn / inv_meff_n;
    jn = fmax(jn,0.0);
    vec2 j_n = VEC2_MUL(n,jn);
    vec2 vt = VEC2_SUB((vec2){v_rel.x,v_rel.y}, VEC2_MUL(n, vn));
    float len_vt = VEC2_LENGTH(vt);
    vec2 j_t=VEC2_ZERO;
    if(len_vt>1e-9){
        vec2 vt_hat=VEC2_MUL(vt, 1/len_vt);
        float rtA = rA.x*vt_hat.y-rA.y*vt_hat.x;
        float rtB = rB.x*vt_hat.y-rB.y*vt_hat.x;
        float inv_meff_t = (inv_ma+inv_mb+inv_ia*rtA*rtA+inv_ib*rtB*rtB);
        float jt = -len_vt /inv_meff_t;
        float max_f= 0.5 * jn;
        jt = clamp(-max_f, max_f, jt);
        j_t = VEC2_MUL(vt_hat, jt);
    }
    vec2 impulse_a=VEC2_ADD(j_n, j_t);
    vec2 impulse_b=VEC2_SUB(VEC2_ZERO, impulse_a);
    info->body_a->linear_velocity = VEC2_ADD(info->body_a->linear_velocity, VEC2_MUL(impulse_a, inv_ma));
    info->body_b->linear_velocity = VEC2_ADD(info->body_b->linear_velocity, VEC2_MUL(impulse_b, inv_mb));
    info->body_a->angular_velocity.z+=inv_ia*(rA.x*impulse_a.y-rA.y*impulse_a.x);
    info->body_b->angular_velocity.z+=inv_ib*(rB.x*impulse_b.y-rB.y*impulse_b.x);
    // position fix
    info->body_a->linear_velocity= VEC2_ADD(info->body_a->linear_velocity, VEC2_MUL(n, info->depth*0.5f));
    info->body_b->linear_velocity= VEC2_SUB(info->body_b->linear_velocity, VEC2_MUL(n, info->depth*0.5f));
}