// CREDITS: Daniel J. Cucuzza
// DATE: February 19th, 2023
// You can contact me at gaunletgames@gmail.com if you have
// any questions about the implementation or if you notice
// any errors.

// Minkowski-Swept GJK. This essentially approximates the time of
// impact of the intersection between two convex sets. You can think of
// this as a generalized raycast for arbitrary (convex) shapes.
// sa := convex set A being traced to B
// sb := convex set B being traced against
// sv := linear swept/traced movement
const CAST_DGJK=(sa,sv,sb,eps=8)=> {
	const org = sa.origin();
// adds to transformation matrix offset
	const ADD_T=(sm,wx=0,wy=0)=> { sm[6]+=wx; sm[7]+=wy; }
// reset transformation matrix
	const RESET_T=(sm)=> { sm[6]=org._x; sm[7]=org._y; }
// total time of impact
	let toi = 0.0;
// cache original primal magnitude
	const nv_l = norm2(sv) + backstep;
// iterative distance which we'll try to minimize to zero
	let dis = nv_l;
// iteration count to avoid over-shooting
	let i = 0;
// squared separation norm
	let nv2 = 0;
// normalize vector
	sv = unit2(sv);
	let query = null;
	let bound = false;

	const splx = new GJKSimplex2D();
	do {
// run the dgjk
		query = DGJK(sa,sb,splx);

// squared norm of minmized separation
		nv2 = dot2(query.nv,query.nv);
// 'pseudo-contact' pair
		const pair = DGJK_CLOSEST(query.splx, query.nv, sa.l2w(), sb.l2w());
// 'pseudo-contact' minmized separation
		const dif = sub2(pair.b, pair.a);
// dis*dot2(dif,sv) > nv2 => velocity vector's projection onto the
// separating axis is further. means we could intersect
		const d1 = dot2(dif,sv);
		if(dis*d1 > nv2) {
// project normalized sv onto difference vector.
			const dlen = Math.sqrt(nv2);
// update world offset matrix
			ADD_T(sa.l2w(), sv._x*dlen, sv._y*dlen);
// subtract halfspace distance
			dis = dis - dlen;
			dis = dis > 0 ? dis : 0;
			toi += dlen;
		}else {
			RESET_T(sa.l2w());
			if(d1 > 0.1 || d1 < -0.1) { 
				bound = true; // we are touching
			}
			break;
		}
	i++;
// epsilon check otherwise we'd take fifty iterations to converge 1/10th of a pixel.
	}while(nv2 > eps);
	RESET_T(sa.l2w());

	return {toi:toi / nv_l, query:query, bound:bound }; // return time of impact
}

// input: two convex sets in the form of point clouds
// (requirements of input sets):
// 1. l2w() is a function returning the local to world matrix 
// transformation. It must be a column major 3x3 matrix!
//
// 2. l2w_iv() is the world to local matrix transformation. It
// must also be a column major 3x3 matrix. It MUST correspond
// to the inverse of whatever is returned in l2w().

// 3. pts() is a member function of your polygon returning an array
// of vec2s that correspond to points!

// Check out the vector.js file for more information. If you're super 
// concerned about performance or want to use your own vector library you 
// can inject your own. That'll take time though. I made the decision for 
// a immutable vec2 out of convenience and not wanting to stick to arrays. 
// Feel free to criticize me, I know its a pain for parity :(.

// (optional) the last simplex iteration if frame
// coherence is high enough to warrant it.
// eps -> epsilon, keep default if you don't know what you're
// doing.
const DGJK=(sa,sb,splx=null,eps=0.0001)=> {
	const pts_a = sa.pts();   // point set array for A
	const pts_b = sb.pts();	  // point set array for B
	const m_a = sa.l2w();     // l2w matrix for A
	const m_b = sb.l2w(); 	  // l2w matrix for B 
	const mi_a = sa.l2w_iv(); // w2l matrix for A
	const mi_b = sb.l2w_iv(); // w2l matrix for B
// helper function returning the translational component of the last column
// in a 3x3
	const origin=(mat)=> {
		const w = mat[8];
		return new vec2(w*mat[6],w*mat[7]);
	}

	const is_dupe=(w,splx)=> {
		for(let i=0;i<splx.dim();i++) {
			const df = sub2(splx.peek(i).ab(),w.ab());
			if(dot2(df,df)< 0.1) return true;
		}
		return false;
	}

// starting point should exist in convex difference
	const nv = sub2(origin(m_a), origin(m_b));
	const d = copy2(nv);

	if(!splx) splx = new GJKSimplex2D();
	else {
// recompute differences (assuming they have not been reassigned to)
// if you manually reassign points in your polygon this simplex will not be
// updated! In place modifications will preserve this attachment.
		splx.rebind(m_a,m_b);
		MUTATE_SIMPLEX(splx,nv);
	}
	let i = 0;
	do {
// flip next vertex to maximize in opposite direction
		d._x = -nv._x; d._y = -nv._y;
		const a = DGJK_SUPPORT(d,  pts_a, mi_a); // maximize v along A
		const b = DGJK_SUPPORT(nv, pts_b, mi_b); // minimize v along B
		const w = new MinkowskiVertex2D(a,b, sub2(MT(m_a,a), MT(m_b,b)));
// floating point heuristic from the help of Gino Van's analysis
		const v1 = nv._x*nv._x + nv._y*nv._y;
		if(is_dupe(w,splx) || v1 - dot2(w.ab(),nv) < eps * v1) {
			break;
		}else {
			splx.push(w);
			MUTATE_SIMPLEX(splx,nv);
		}
	} while(i++ < (pts_a.length + pts_b.length) && splx.dim() < 3);
	return { splx, nv };
}
// designed to run on the finishing simplex of the DGJK. Computes the
// barycentric decomposition of a point in standard coordinates.
// this is super easy in 2D as we only need to deal with vector projections.
const DGJK_CLOSEST=(splx,nv,m_a,m_b)=> {
	switch(splx.dim()) {
		default:
			return {a:zero2(),b:zero2()};
		case 1:
			const p1 = splx.peek(0);
			return {a:MT(m_a,p1.a()),b:MT(m_b,p1.b())};
		case 2:
		case 3:
			return BARY_2(splx,nv,m_a,m_b);
	}
}
// designed alongside DGJK_CLOSEST(...), DGJK_NORMAL() computes a 
// separating axis for the two convex sets
const DGJK_NORMAL=(splx,sv,m_a,m_b)=> {
	if(splx.dim() == 1) {
		const a = splx.peek(0);
		return unit2(a.ab());
	}else if(splx.dim() == 2) {
		const a = splx.peek(0);
		const b = splx.peek(1);
		const ab = unit2(perp2(sub2(a.ab(),b.ab())));
		if(dot2(ab,sv) > 0) { ab._x *= -1; ab._y *= -1; }
		return ab;
	}else if(splx.dim() == 3) {
		const a = splx.peek(0);
		const b = splx.peek(1);
		let ab = perp2(sub2(a.ab(),b.ab()));
		if(dot2(ab,sv) > 0) { ab._x *= -1; ab._y *= -1; }
		return unit2(ab);
	}	
	return zero2();
}

// barycentric coordinates for one dimensional
// simplices (its as easy as a simple vector projection)
const BARY_2=(splx,nv,m_a,m_b)=> {
	const p1 = splx.peek(0);
	const p2 = splx.peek(1);
	let ab = sub2(p2.ab(),p1.ab());

	let t = dot2(ab,sub2(nv,p1.ab())) / dot2(ab,ab);
	t = clamp(t,0,1);
	return {
		a:MT(m_a,lerp2(p1.a(),p2.a(),t)),
		b:MT(m_b,lerp2(p1.b(),p2.b(),t))
	};
}

// mutate the simplex to eliminate any facets not
// contributing to a convex combination of nv.
const MUTATE_SIMPLEX=(splx,nv)=> {
	switch(splx.dim()) {
		case 1: // convex, simple point (0D)
			MUTATE_0D(nv,splx);
		break;
		case 2: // convex, simple segment (1D)
			MUTATE_1D(nv,splx);
		break;
		case 3: // convex, simple area (2D)
			MUTATE_2D(nv,splx);
		break;
	}
}
// responsible for handling simplex state for 
// a zero dimensional simplex (point)
const MUTATE_0D=(nv,splx)=> {
	const a = splx.peek(0).ab();
	nv._x = a._x; nv._y = a._y;
}
// responsible for handling simplex state for
// a one dimensional simplex (convex segment)
const MUTATE_1D=(nv,splx)=> {
	const a = splx.peek(0);
	const b = splx.peek(1);
	const rep = VOROCENTRIC_1D({a:a.ab(), b:b.ab()}, {a:0x1, b:0x2});
	nv._x = rep.v._x; nv._y = rep.v._y;
	switch(rep.r) {
		case 0x1: // A
			const a = splx.peek(0);
			splx.clear();
			splx.push(a);
		break;
	}
}
// responsible for handling simplex state for
// a two dimensional simplex (convex triangle)
const MUTATE_2D=(nv,splx)=> {
	const a = splx.peek(0);
	const b = splx.peek(1);
	const c = splx.peek(2);
	const rep = VOROCENTRIC_2D({a:a.ab(),b:b.ab(),c:c.ab()},{a:0x1,b:0x2,c:0x4});
	nv._x = rep.v._x; nv._y = rep.v._y;
	switch(rep.r) { 
// A contributes
		case 0x1: splx.clear(); splx.push(a);
		break;
// B contributes
		case 0x2: splx.clear(); splx.push(b);
// BA contributes
		case 0x3: splx.clear(); splx.push(b); splx.push(a);
		break;
// C contributes
		case 0x4: splx.clear(); splx.push(c);
		break;
// CA contributes
		case 0x5: splx.clear(); splx.push(c); splx.push(a);
		break;
// BC contributes
		case 0x6: splx.clear(); splx.push(c); splx.push(b);
		break;
	}
}
// Barycentric/Voronoi decomposition for a 
// line segment in R^2.
const VOROCENTRIC_1D=(edge,bits)=> {
	const ao = mul2(-1, edge.a);
	const bo = mul2(-1, edge.b);
	const ab = sub2(edge.b, edge.a);

	const abm = norm2(ab);
	if(abm > 0) {
		ab._x /= abm;
		ab._y /= abm;
	}

	const v = dot2(ao, ab);
	if(v > abm) {
		return {r:bits.b, v:edge.b};
	}else if(v < 0) {
		return {r:bits.a, v:edge.a};
	}else {
		return {r:bits.a | bits.b,
			v:new vec2(
				edge.a._x + ab._x*v,
				edge.a._y + ab._y*v
			),
		}
	}
}
// Barycentric/Voronoi decomposition for a
// triangle in R^2.
const VOROCENTRIC_2D=(tri,bits)=> {
	const o = zero2();
	const a = tri.a;
	const b = tri.b;
	const c = tri.c; 
	const same=(v,w)=> {
		return dot2(v,w) > 0;
	}
// computes the signed orthogonal projections onto each
// combinational segment on the simplex
	const det = DET_2D(sub2(a,c),sub2(a,b));
	const signed_areas=(det)=> {
		let nflags = 0;
		nflags |= ORIENT_2D(a,b,o,det) ? 0x1 : 0; // BA
		nflags |= ORIENT_2D(b,c,o,det) ? 0x2 : 0; // CB
		nflags |= ORIENT_2D(c,a,o,det) ? 0x4 : 0; // AC 
		return nflags;
	}
	const dual_edges=(pm,bits)=> {
		const xy = sub2(pm.y, pm.x);
		const zy = sub2(pm.y, pm.z);

		const xp = mul2(-1, pm.x);
		const yp = mul2(-1, pm.y);
		const zp = mul2(-1, pm.z);
// REGION Y
		if(same(yp,zy) && same(yp, xy)) {
			return {r:bits.y, v:pm.y};
		}
		if(!same(yp,xy)) {
			if(!same(xp,xy)) {
// REGION X
				return {r:bits.x, v:pm.x};
			}else {
// REGION XY
				return {r:bits.x|bits.y, 
					v:add2(onto2c(xp, xy), pm.x),
				};
			}
		}
		if(!same(yp, zy)) {
// REGION Z
			if(!same(zp,zy)) {
				return {r:bits.z, v:pm.z};
			}else {
// REGION ZY
				return {r:bits.y|bits.z,
					v:add2(onto2c(zp, zy), pm.z)
				};
			}
		}
		return {r:bits.x|bits.y|bits.z, v:zero2()};
	}
	switch(signed_areas(det)) {
		case 1: // AB-edge
			return VOROCENTRIC_1D(
				{a:tri.a, b:tri.b}, {a:bits.a,b:bits.b});
		case 2: // CB-edge
			return VOROCENTRIC_1D(
				{a:tri.c,b:tri.b}, {a:bits.c,b:bits.b});
		case 3: // ABC-chain
			return dual_edges(
					{x:tri.a,y:tri.b,z:tri.c},{x:bits.a,y:bits.b,z:bits.c}
				);
		case 4: // CA-chain
			return VOROCENTRIC_1D(
				{a:tri.c,b:tri.a},{a:bits.c,b:bits.a}
			);
		case 5: // CAB-chain
			return dual_edges(
				{x:tri.c,y:tri.a,z:tri.b}, {x:bits.c,y:bits.a,z:bits.b}
			);
		case 6: // BCA-chain
			return dual_edges(
				{x:tri.b,y:tri.c,z:tri.a}, {x:bits.b,y:bits.c,z:bits.a}
			);
		default: // inscribed => intersection. will recurse @ top
			return {r:bits.a|bits.b|bits.c, v:zero2()};
	}
}
// optimize later, focus on getting the support function
// to run, work on greedy adjacency approach using DCEL
// using simulation coherency, this can be a near constant
// time operation if correctly implemented. As of now, we
// naively search the whole set for its maximum.
const DGJK_SUPPORT=(v,set,inv)=> {
	let mv = set[0];
	let max = Number.NEGATIVE_INFINITY;
// invert direction w.r.t coordinate frame
	tv = mTransform3x3(inv, [v._x,v._y,0]);
	for(let i=0;i<set.length;i++) {
		const dot = set[i]._x*tv[0] + set[i]._y*tv[1];
		if(dot >= max) {
			max = dot;
			mv = set[i];
		}
	}
	return mv;
}

// A Minkowski Vertex is a vertex containing the vertex
// of convex sets A, and B and their difference. These will all
// be used in calculating barycentric coordinates for closest
// point computations.
class MinkowskiVertex2D {
	#_a; #_b; #_ab;
	constructor(a,b,ab) {
		this.#_a = a;
		this.#_b = b;
		this.#_ab = ab;
	}
	a=()=>this.#_a;
	b=()=>this.#_b;
	ab=()=>this.#_ab;
	rebind=(ab)=>{this.#_ab = ab; }
}

// d-simplex restricted to two dimensions for the GJK algorithm. This will
// function similar to that of a stack as the elements will be reordered.
class GJKSimplex2D {
	#_pts; #_dim;
	constructor() {
// dimensionality of simplex
		this.#_dim = 0;
// pseudo-stack arraylist
		this.#_pts = [];
	}
// requires support points a and b
	push=(w)=> {
		if(this.#_dim < 3) {
			this.#_pts.push(w);
			this.#_dim++;
// swap
			for(let i=this.#_dim-1;i>0;i--) {
				const c = this.#_pts[i];
				const p = this.#_pts[i-1];
				this.#_pts[i-1]=c;
				this.#_pts[i]=p;
			}
		}else { return; }
	}
	peek=(i)=> {
		if(i > this.#_dim) { return null; }
		else { return this.#_pts[i]; }
	}
	pop=()=> {
		if(this.#_dim > 0) { return this.#_pts[this.#_dim--]; }
		else return null;
	}
	clear=()=> {
		this.#_dim = 0;
		while(this.#_pts.length > 0) this.#_pts.pop();
	}
	dim=()=> { return this.#_dim; }
	rebind=(m_a,m_b)=> { 
		for(let i=0;i<this.#_pts.length;i++) { 
			this.#_pts[i].rebind(
				sub2(MT(m_a,this.#_pts[i].a()),
					 MT(m_b,this.#_pts[i].b())
			));
		}
	}
}
