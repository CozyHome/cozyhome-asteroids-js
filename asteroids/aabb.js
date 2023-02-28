// storage container of the center, extent OBB datastructure. This will be used as a blank
// slate for other systems to use themselves. I am avoiding implementing any state
// in the object itself for ease of accessibility -DC @ 2/22/2023
class OBB {
	#_cen; #_ext;
	constructor(min,max) {
		this.#_cen = new vec2(
			(min._x + max._x) / 2,
			(min._y + max._y) / 2
		);
		this.#_ext = new vec2(
			(max._x - min._x) / 2,
			(max._y - min._y) / 2
		);
		if(this.#_ext._x < 0) this.#_ext._x *= -1;
		if(this.#_ext._y < 0) this.#_ext._y *= -1;
	}
	cen=()=>{ return this.#_cen; }
	ext=()=>{ return this.#_ext; }
}

// storage container of the min-max AABB datastructure. This will be used as a blank
// slate for other systems to use themselves. I am avoiding implementing any state
// in the object itself for ease of accessibility -DC @ 2/22/2023
class AABB {
	#_cen; #_ext;
	constructor(minx=0,maxx=0,miny=0,maxy=0) {
		this.#_cen = new vec2();
		this.#_ext = new vec2();
		this.#_cen._x = (minx + maxx) / 2;
		this.#_cen._y = (miny + maxy) / 2;
		this.#_ext._x = (maxx - minx) / 2;
		this.#_ext._y = (maxy - miny) / 2;
		if(this.#_ext._x < 0) this.#_ext._x *= -1;
		if(this.#_ext._y < 0) this.#_ext._y *= -1;
	}
	cen=()=> { return this.#_cen; }
	ext=()=> { return this.#_ext; }
	min=()=> { return sub2(this.#_cen, this.#_ext); }
	max=()=> { return add2(this.#_cen, this.#_ext); }
	bind=(minx,maxx,miny,maxy)=> {
		this.#_cen._x = (minx + maxx) / 2;
		this.#_cen._y = (miny + maxy) / 2;
		this.#_ext._x = (maxx - minx) / 2;
		this.#_ext._y = (maxy - miny) / 2;
		if(this.#_ext._x < 0) this.#_ext._x *= -1;
		if(this.#_ext._y < 0) this.#_ext._y *= -1;
	}
}

// takes an OBB and returns an AABB
const AABB_FROM_OBB=(obb, m)=> {
	const aabb = new AABB();
	UPDATE_AABB_OBB(aabb, obb, m);
	return aabb;
}

// takes an OBB and updates the aabb
const UPDATE_AABB_OBB=(aabb, obb, m)=> {
	const wcen = MT(m, obb.cen());
	const wext = MTD_ABS(m, obb.ext());

	const minx = wcen._x - wext._x;
	const maxx = wcen._x + wext._x;
	const miny = wcen._y - wext._y;
	const maxy = wcen._y + wext._y;
	
	aabb.bind(minx,maxx,miny,maxy);
}

// duplicates an AABB
const COPY_AABB=(A)=> {
	const max = A.max();
	const min = A.min();
	return new AABB(min._x,max._x,min._y,max._y);
}

// copies A into B
const COPY_TO_AABB=(A,B)=> {
	const cen_a = A.cen();
	const ext_a = A.ext();
	const cen_b = B.cen();
	const ext_b = B.ext();
	cen_b._x = cen_a._x; cen_b._y = cen_a._y;
	ext_b._x = ext_a._x; ext_b._y = ext_a._y;
}

// takes a convex polygonal set and constructs its axis aligned bounding box via
// an inverse matrix transformation of axial directions
const CONVEX_OBB=(pts)=> {
	const sup = new vec2(0,1);
	const m_dn = SUPPORT_OBB(sup,pts); // down
	sup._y = -1; 
	const m_up = SUPPORT_OBB(sup,pts); // up
	sup._x = 1; sup._y = 0;
	const m_rt = SUPPORT_OBB(sup,pts); // right
	sup._x = -1;
	const m_lt = SUPPORT_OBB(sup,pts); // left

	const min = new vec2(Math.min(m_lt._x, m_rt._x), Math.min(m_dn._y, m_up._y));
	const max = new vec2(Math.max(m_lt._x, m_rt._x),Math.max(m_dn._y, m_up._y));
	return new OBB(min,max);
}

// takes a convex polygonal set and constructs its axis aligned bounding box via
// an inverse matrix transformation of axial directions
const CONVEX_AABB=(pts, m, m_i)=> {
	const sup = new vec2(0,1);
	const m_dn = MT(m, SUPPORT_AABB(sup,pts,m_i)); // down
	sup._y = -1; 
	const m_up = MT(m, SUPPORT_AABB(sup,pts,m_i)); // up
	sup._x = 1; sup._y = 0;
	const m_rt = MT(m, SUPPORT_AABB(sup,pts,m_i)); // right
	sup._x = -1;
	const m_lt = MT(m, SUPPORT_AABB(sup,pts,m_i)); // left

	return new AABB(Math.min(m_lt._x, m_rt._x), Math.max(m_lt._x, m_rt._x),
		Math.min(m_dn._y, m_up._y), Math.max(m_dn._y, m_up._y)
	);
}

const SUPPORT_OBB=(v,set)=> {
	let mv = set[0];
	let max = Number.NEGATIVE_INFINITY;
	for(let i=0;i<set.length;i++) {
		const dot = set[i]._x*v._x + set[i]._y*v._y;
		if(dot >= max) {
			max = dot;
			mv = set[i];
		}
	}
	return mv;
}

const SUPPORT_AABB=(v,set,inv)=> {
	let mv = set[0];
	let max = Number.NEGATIVE_INFINITY;
// invert direction w.r.t coordinate frame
	tv = mTransform(inv, [v._x,v._y,0]);
	for(let i=0;i<set.length;i++) {
		const dot = set[i]._x*tv[0] + set[i]._y*tv[1];
		if(dot >= max) {
			max = dot;
			mv = set[i];
		}
	}
	return mv;
}

const CCD_AABB=(v, box, inf=8)=> {
	const cen = box.cen(); const ext = box.ext();
	const abs=(a)=>{ return a > 0 ? a : -a; }
	cen._x += v._x/2; cen._y += v._y/2;
	ext._x += inf + abs(v._x/2); ext._y += inf + abs(v._y/2);
}

const TEST_AABB_AABB=(A,B,slack=0)=> {
	const abs=(a)=>{ return a > 0 ? a : -a; }
	const ac = A.cen(); const ar = A.ext();
	const bc = B.cen(); const br = B.ext();

	if(abs(ac._x - bc._x) > ar._x + br._x + slack) return false;
	if(abs(ac._y - bc._y) > ar._y + br._y + slack) return false;
	return true;
}

// split AABB into four quadrants
const SPLIT_AABB=(box)=> {
	const ext = box.ext();
	const bmin = box.min();
	const bmax = box.max();
// top left
	const tl_box = new AABB(bmin._x, bmin._x + ext._x, bmin._y, bmin._y + ext._y);
// top right
	const tr_box = new AABB(bmax._x - ext._x, bmax._x, bmin._y, bmin._y + ext._y);
// bottom left
	const bl_box = new AABB(bmin._x, bmin._x + ext._x, bmax._y - ext._y, bmax._y);
// bottom right
	const br_box = new AABB(bmax._x - ext._x, bmax._x, bmax._y - ext._y, bmax._y);

	return [tl_box, tr_box, br_box, bl_box];
}

// simple draw call for local space OBB (requires matrix to transform)
const DRAW_OBB=(m,obb)=> {
	push();
	PUSH_P5(m);
		const cen = obb.cen(); const ext = obb.ext();
		cir2(obb.cen(),16);	
		line(cen._x - ext._x, cen._y - ext._y, cen._x - ext._x, cen._y + ext._y);
		line(cen._x - ext._x, cen._y + ext._y, cen._x + ext._x, cen._y + ext._y);
		line(cen._x + ext._x, cen._y + ext._y, cen._x + ext._x, cen._y - ext._y);
		line(cen._x + ext._x, cen._y - ext._y, cen._x - ext._x, cen._y - ext._y);
	draw2p(cen,ext);
	pop();
}

// simple draw call for worldspace AABB
const DRAW_AABB=(aabb)=> {
	const cen = aabb.cen();
	const ext = aabb.ext();
	line(cen._x - ext._x, cen._y - ext._y, cen._x - ext._x, cen._y + ext._y);
	line(cen._x - ext._x, cen._y + ext._y, cen._x + ext._x, cen._y + ext._y);
	line(cen._x + ext._x, cen._y + ext._y, cen._x + ext._x, cen._y - ext._y);
	line(cen._x + ext._x, cen._y - ext._y, cen._x - ext._x, cen._y - ext._y);
}
