#include "collision.h"
#include "body.h"
#include "geometry.h"
#include "shape.h"
#include "vec.h"


int generate_contact_info(body*body_a,body*body_b,contact_info* info){
    uint8_t type=body_a->shape.id |body_b->shape.id;
    if(type == (TYPE_CIRCLE | TYPE_CIRCLE)){
        /* circle centre is the body position; do NOT cache a pointer to it,
         * the bodies array can be reallocated at any time */
        vec2 ca=body_a->position;
        vec2 cb=body_b->position;
        if(!circle_contact_circle(ca, cb, body_a->shape.c.r, body_b->shape.c.r))return -1;
        /* normal points from B to A, matching solve_contact's sign convention */
        vec2 n=VEC2_NORMALIZE(VEC2_SUB(ca, cb));
        info->normal=n;
        info->tangent=VEC2_PERP(n);
        info->body_a = body_a;
        info->body_b = body_b;
        float dist = VEC2_LENGTH(VEC2_SUB(cb,ca));
        /* contact point on A's surface facing B (dir is A->B) */
        vec2 dir=VEC2_MUL(n,-1.0f);
        info->collision_point=VEC2_ADD(ca, VEC2_MUL(dir, body_a->shape.c.r));
        info->depth = body_a->shape.c.r+body_b->shape.c.r-dist;
        return 0;
    }
    else if(type == (TYPE_CIRCLE | TYPE_SEGMENT)){
        body* c= (body_a->shape.id==TYPE_CIRCLE)?body_a:body_b;
        body* l= (c==body_a)?body_b:body_a;
        vec2 cp=c->position;

        float cr=c->shape.c.r;
        vec2 lp1=VEC2_ADD(l->position, l->shape.s.p1);
        vec2 lp2=VEC2_ADD(l->position, l->shape.s.p2);
        if (point_segment_distance(lp1, lp2, cp)>cr)return -1;
        vec2 v=point_segment_vector(lp1, lp2, cp);
        info->collision_point=VEC2_SUB(cp,v);
        info->body_a=c;
        info->body_b=l;
        info->depth=fabsf(cr-VEC2_LENGTH(v));
        info->normal=VEC2_NORMALIZE(v);
        info->tangent=VEC2_PERP(VEC2_NORMALIZE(v));
        return 0;
    }
    return -1;
}

/* ---- contact solver tuning ----------------------------------------------
 * CONTACT_RESTITUTION: 1.0 = perfectly elastic (bounces forever, stacks never
 *                      settle). Lower it to bleed off energy.
 * CONTACT_RESTITUTION_THRESHOLD: contacts approaching slower than this get no
 *                      restitution at all. Without it a resting stack keeps
 *                      micro-bouncing and never stops.
 * CONTACT_SLOP:        penetration tolerated before position recovery kicks
 *                      in; stops stacks from jittering on tiny overlaps.
 * CONTACT_BETA:        fraction of the penetration recovered per step. Smaller
 *                      = gentler position recovery.
 * CONTACT_MAX_CORRECTION: how much penetration one step may try to remove, so
 *                      a deep overlap can never blow up.
 * CONTACT_FRICTION:    Coulomb friction coefficient.
 */
#ifndef CONTACT_RESTITUTION
#define CONTACT_RESTITUTION           1.0f
#endif
#ifndef CONTACT_RESTITUTION_THRESHOLD
#define CONTACT_RESTITUTION_THRESHOLD 1.0f
#endif
#ifndef CONTACT_SLOP
#define CONTACT_SLOP                  0.5f
#endif
#ifndef CONTACT_BETA
#define CONTACT_BETA                  0.8f
#endif
#ifndef CONTACT_MAX_CORRECTION
#define CONTACT_MAX_CORRECTION        5.0f
#endif
#ifndef CONTACT_FRICTION
#define CONTACT_FRICTION              0.5f
#endif

/* Velocity pass: restitution + friction only. It never adds a position bias,
 * so it cannot pump energy into the simulation. */
void solve_contact(contact_info* info, float dt){
    (void)dt;
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

    float rnA = rA.x*n.y-rA.y*n.x;
    float rnB = rB.x*n.y-rB.y*n.x;
    float inv_meff_n = (inv_ma + inv_mb + inv_ia * rnA * rnA + inv_ib * rnB * rnB);
    if (inv_meff_n==0)return;

    /* Only bounce for genuine impacts. Below the threshold treat the contact
     * as resting (e = 0) so a stack can actually come to rest. */
    float e = (vn < -CONTACT_RESTITUTION_THRESHOLD) ? CONTACT_RESTITUTION : 0.0f;

    /* normal impulse: drive the relative normal velocity to -e*vn.
     * clamp to >=0 so contacts only push, never pull. */
    float jn = (-(1.0f + e)*vn) / inv_meff_n;
    if (jn < 0.0f) jn = 0.0f;
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
        float max_f= CONTACT_FRICTION * jn;
        jt = clamp(-max_f, max_f, jt);
        j_t = VEC2_MUL(vt_hat, jt);
    }
    vec2 impulse_a=VEC2_ADD(j_n, j_t);
    vec2 impulse_b=VEC2_SUB(VEC2_ZERO, impulse_a);
    info->body_a->linear_velocity = VEC2_ADD(info->body_a->linear_velocity, VEC2_MUL(impulse_a, inv_ma));
    info->body_b->linear_velocity = VEC2_ADD(info->body_b->linear_velocity, VEC2_MUL(impulse_b, inv_mb));
    info->body_a->angular_velocity.z+=inv_ia*(rA.x*impulse_a.y-rA.y*impulse_a.x);
    info->body_b->angular_velocity.z+=inv_ib*(rB.x*impulse_b.y-rB.y*impulse_b.x);
}

/* Position pass (split impulse): recover overlap with separate pseudo
 * velocities. This is the "velocity change" that pushes overlapping bodies
 * apart, but because it is applied to body->position_bias instead of the real
 * velocity, it never adds or removes energy. That is what lets a stack settle
 * instead of jittering forever. */
void solve_position(contact_info* info, float dt){
    vec2 n=info->normal;
    body* a=info->body_a;
    body* b=info->body_b;
    float inv_ma=1/a->mass;
    float inv_mb=1/b->mass;
    float inv_ia=1/a->inertia;
    float inv_ib=1/b->inertia;
    vec2 rA=VEC2_SUB(info->collision_point, a->position);
    vec2 rB=VEC2_SUB(info->collision_point, b->position);

    /* relative pseudo velocity at the contact point (2d cross: w x r = w*(-r.y, r.x)) */
    vec2 vA=VEC2_ADD(a->position_bias, (vec2){-a->rotation_bias*rA.y, a->rotation_bias*rA.x});
    vec2 vB=VEC2_ADD(b->position_bias, (vec2){-b->rotation_bias*rB.y, b->rotation_bias*rB.x});
    float vn=VEC2_DOT(VEC2_SUB(vA,vB), n);

    float rnA = rA.x*n.y-rA.y*n.x;
    float rnB = rB.x*n.y-rB.y*n.x;
    float inv_meff_n = (inv_ma + inv_mb + inv_ia * rnA * rnA + inv_ib * rnB * rnB);
    if (inv_meff_n==0)return;

    /* target separating pseudo velocity; a fraction beta of the penetration
     * beyond the slop, clamped so a deep overlap stays bounded. Integrating
     * this pseudo velocity over dt moves the bodies apart by beta*pen. */
    float pen = info->depth - CONTACT_SLOP;
    if (pen < 0.0f) pen = 0.0f;
    if (pen > CONTACT_MAX_CORRECTION) pen = CONTACT_MAX_CORRECTION;
    float bias = (CONTACT_BETA/dt) * pen;

    float jn = (bias - vn) / inv_meff_n;
    if (jn < 0.0f) jn = 0.0f;
    vec2 P=VEC2_MUL(n,jn);

    a->position_bias=VEC2_ADD(a->position_bias, VEC2_MUL(P, inv_ma));
    b->position_bias=VEC2_SUB(b->position_bias, VEC2_MUL(P, inv_mb));
    a->rotation_bias += inv_ia*(rA.x*P.y-rA.y*P.x);
    b->rotation_bias -= inv_ib*(rB.x*P.y-rB.y*P.x);
}
