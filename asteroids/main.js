// CREDITS: Daniel J. Cucuzza
// DATE: February 19th, 2023
// You can contact me at gaunletgames@gmail.com if you have
// any questions about the implementation or if you notice
// any errors.

let CAM_M;
let N = 1;
// polygon entity constructor
const POLYGON_ECT=(n=7,w=64,h=64)=> {
	const gon = RANDOM_CONVEX_POLYGON(n,w,h);
	gon.bind=(props)=> { gon._uid = props.id; }
	gon.uid=()=>{ return gon._uid; }

	let vr1 = random();
	let vr2 = random();

	gon.vel = new vec2(-128*(1-vr1) + vr1*128, -128*(1-vr2) + vr2*128 );
// powerhouse of the simulation: 
	gon.ccd_aabb = COPY_AABB(gon.aabb());
	gon.hits = [];
	gon.move=(man,dt)=> {
// don't bother moving
		const m = gon.l2w();
		const v = gon.vel;
		const dv = mul2(dt, v);
		const ccd = gon.ccd_aabb;
		COPY_TO_AABB(gon.aabb(), ccd);
		CCD_AABB(dv, ccd);
		stroke(0,255,0); DRAW_AABB(ccd);
		stroke(255); DRAW_AABB(gon.aabb());
		QUERY_QUADTREE(man.qtree, ccd, gon.uid(), gon.hits);
		const hits = gon.hits;

		if(hits.length > 0) {
			v._x *= -1; v._y *= -1;
		}else {
			m[6] += dv._x; m[7] += dv._y;
		}

		REMOVE_QUADTREE(man.qtree, gon);
		UPDATE_AABB_OBB(gon.aabb(), gon.obb(), m);
		INSERT_QUADTREE(man.qtree, gon);
	}
	return gon;
}

// returns randomized DCEL and cloud representation for convex
// polygon in R^2.
const RANDOM_CONVEX_POLYGON=(n=7,w=64,h=64)=> {
// randomized point in R^2 scaled w.r.t AABB
	const rpoint=(w,h)=> { 
		return new vec2(w*random(),h*random());
	}
	const pts=[];
	for(let i=0;i<n;i++) {
		pts.push(rpoint(w,h));
	}
	return CONVEX_POLYGON(pts);
}

// This is an example generating function you could use
// to have objects compatible with DGJK:
const CONVEX_POLYGON=(pts)=>{
// get the convex hull of an arbitary point set in R^2
	const gon = QUICKHULL(pts);
	const hull = gon.hull;

// compute centroid to then use as origin for object space
// representation
	const centroid = zero2();
	for(let i=0;i<hull.length;i++) {
		centroid._x += hull[i]._x;
		centroid._y += hull[i]._y;
	}
// centroid is a uniform distribution of weight along
// all contributing points
	centroid._x /= hull.length;
	centroid._y /= hull.length;		
// reconfigure polygon to be w.r.t the centroid of its 
// vertices. This essentially allows us to apply linear transformations
// from the viewpoint of the polygon's centroid.
	for(let i=0;i<hull.length;i++) {
		hull[i]._x = hull[i]._x - centroid._x;
		hull[i]._y = hull[i]._y - centroid._y;
	}

	gon.mat = new Matrix3x3();
	gon.mat.translate(centroid._x, centroid._y);
// set up the matrix dependencies for the polygon
// l2w() and l2w_iv() are REQUIRED for DGJK to function. These must
// be part of your input set to function.
	gon.l2w =()=> { return gon.mat.get(); }				// local to world matrix accessor
	gon.l2w_iv=()=> { return mInverse3x3(gon.l2w()); }		// world to local matrix accessor
	gon.pts =()=> { return gon.hull; }					// point set accessor
// helper method for center calculation (not necessarily needed for DGJK to run)
	gon.origin=()=> {
		const m = gon.l2w();
		const w = m[8];
		return new vec2(w*m[6],w*m[7]);
	}
	
	gon._obb = CONVEX_OBB(gon.hull);
	gon._aabb = AABB_FROM_OBB(gon._obb, gon.l2w());
	gon.aabb =()=> { return gon._aabb; }
	gon.obb =()=> { return gon._obb; }
	return gon;
}


let GJK_E; 		// program entity
let ENT_LIST;

const GJK_FSM = new FSM([{
	key:'init',
	setup:function(fsm,man) {},
	enter:function(prev,fsm,man) {
		fsm.cswitch(man, 'gjk');
	},
	exit:function(next,fsm,man) {},
	pulse:function(fsm,man) {}
},
{
	key:'gjk',
	setup:function(fsm,man) {},
	enter:function(prev,fsm,man) {
		man.qtree = INIT_QUADTREE(-width,width,-height,height);
		ENT_LIST = new ObjectList(new UIDHandler(), {});
		CAM_M = mTranslate3x3(-width/2,-height/2);

		man.onClick=(mv)=> {
			if(mouseButton == LEFT) {
				// if(ENT_LIST.count() > 1) return;
				ENT_LIST.write_obj(()=>{
					const ran_gon = POLYGON_ECT(9,64,64); 
					const m = ran_gon.l2w();
					m[6] = mv._x; m[7] = mv._y;
					UPDATE_AABB_OBB(ran_gon.aabb(), ran_gon._obb, m);
					INSERT_QUADTREE(man.qtree, ran_gon);
					return ran_gon;
				}, {});
			}
			else if(mouseButton == CENTER) {
					const last = ENT_LIST.get_obj(ENT_LIST.count()-1);
					if(last) {
						ENT_LIST.rem_obj(last.uid(), ()=> {
							REMOVE_QUADTREE(man.qtree, last);
							last._uid = 0;
						});
					}
				}
		}
	},
	exit:function(next,fsm,man) {},
	pulse:function(fsm,man) {
		noFill();

		PUSH_P5(mInverse3x3(CAM_M));
		strokeWeight(1);
		stroke(255);
		let mv = new vec2(mouseX, mouseY);
		mv = (MT(CAM_M,mv));
		for(let i=1;i<ENT_LIST.length();i++) {
			const ent = ENT_LIST.get_obj(i);
			if(!ent) continue;
			ent.move(man, deltaTime/1000);
			DRAW_POLYGON(ent, i-1);
		}
	
// draw origin
		stroke(0,255,0);
		circle(0,0,16);

		DRAW_QUADTREE(man.qtree);
		let m = mIdentity3x3();
		const sp = 128;

		if(keyIsDown(81)) m = mMultiply3x3(mRotate3x3(0.5*deltaTime/1000), m);
		if(keyIsDown(69)) m = mMultiply3x3(mRotate3x3(-0.5*deltaTime/1000), m);

		CAM_M = mMultiply3x3(CAM_M, m);
		m = mIdentity3x3();

		if(keyIsDown(65)) m = mMultiply3x3(mTranslate3x3(-sp*deltaTime/1000,0), m);
		if(keyIsDown(68)) m = mMultiply3x3(mTranslate3x3(sp*deltaTime/1000,0), m);
		if(keyIsDown(87)) m = mMultiply3x3(mTranslate3x3(0,-sp*deltaTime/1000), m);
		if(keyIsDown(83)) m = mMultiply3x3(mTranslate3x3(0,sp*deltaTime/1000), m);

		CAM_M = mMultiply3x3(CAM_M, m);
		m = mIdentity3x3();

		if(keyIsDown(90)) m = mMultiply3x3(mScale3x3(1 + 2*deltaTime/1000), m);
		if(keyIsDown(88)) m = mMultiply3x3(mScale3x3(1 - 2*deltaTime/1000), m);

		CAM_M = mMultiply3x3(CAM_M, m);
	}
}]);

function preload() {}
function setup() {
	const canvas = createCanvas(600,600);
	canvas.id("p5canvas");
	canvas.parent("#center_flexbox");

	GJK_E = CONSTRUCTOR_FSE(GJK_FSM);
}
function mousePressed() {
	const mv = new vec2(mouseX, mouseY);
//	GJK_E.man.onClick(MT(mInverse3x3(CAM_M),mv));
	GJK_E.man.onClick(MT(CAM_M,mv));
}
function keyTyped() {
//	GJK_E.man.onKey(keyCode, new vec2(mouseX, mouseY));
}
function draw() {
	background(0);
	const mv = new vec2(mouseX, mouseY);
	noFill(); stroke(255); circle(0,0,32);
	GJK_E.fsm.pulse(GJK_E.man);
}
