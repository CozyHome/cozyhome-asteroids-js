const MAX_ENTITIES_PER_LEAF = 4;
const MAX_QUADTREE_DEPTH = 4;

class QuadNode {
	#_ch; 	// children
	#_en; 	// entities
	#_pt; 	// parent
	#_aabb; // axis aligned bounding box
	#_dp;
	constructor(parent, aabb) {
		this.#_ch = []; 	// children array
		this.#_en = []; 	// entity array
		this.#_pt = parent; // parent
		this.#_aabb = aabb; // bounds
		this.#_dp = 0; // assume root
		if(this.#_pt) {
			this.#_dp = this.#_pt.depth() + 1;
		}
	}
	children=()=>{ return this.#_ch; }
	entities=()=>{ return this.#_en; }
	parent=()=>{ return this.#_pt; }
	aabb=()=>{ return this.#_aabb; }
	isleaf=()=>{ return !(this.#_ch.length > 0); }
	depth=()=>{ return this.#_dp; }
}

const DRAW_QUADTREE=(root)=> {
	if(!root) return;
	const queue = new Queue();
	queue.push(root);
	do {
		const pop = queue.pop();
		DRAW_AABB(pop.aabb());
// draw sub-children
		if(!pop.isleaf()) {
			const children = pop.children();
			for(let i=0;i < children.length;i++) queue.push(children[i]);
		}
	}while(!queue.empty());
}

// constructing a quadtree base root
const INIT_QUADTREE=(minx,maxx,miny,maxy)=> {
	const box = new AABB(minx,maxx,miny,maxy);
	return new QuadNode(null, box);
}

const INSERT_QUADTREE=(root,ent)=> {
	if(!root) return;
	const queue = new Queue();
	queue.push(root);
	do {
		const pop = queue.pop();
		const children = pop.children();
		const entities = pop.entities();
		const box = pop.aabb();
// if current node intersects with AABB, get its children
// and further traverse downwards
		if(TEST_AABB_AABB(box, ent.aabb())) {
			if(pop.isleaf()) {
				entities.push(ent);
// length size invalidation may have occurred, bring the tree to next generation if needed.
				EVOLVE_QUADTREE(pop);
			}else {
				for(let i=0;i< children.length;i++) queue.push(children[i]);
			}
		}
	}while(!queue.empty());
}

// modifies the contents of the quadtree given that an AABB may have moved.
// this assumes we can modify the aabb freely.
const REMOVE_QUADTREE=(root,ent)=> {
	if(!root) return;
	const conflicts = [];
	const queue = new Queue();
	queue.push(root);
	do {
		const pop = queue.pop();
		const children = pop.children();
		if(TEST_AABB_AABB(pop.aabb(), ent.aabb())) {
			if(pop.isleaf()) {
				conflicts.push(pop);
			}else {
				for(let i=0;i< children.length;i++) queue.push(children[i]);	
			}
		}
	}while(!queue.empty());
// find all conflicts,  
	for(let i=conflicts.length-1;i>=0;i--) {
		const entities = conflicts[i].entities();
		for(let j=entities.length-1;j>=0;j--) {
			const c_ent = entities[j];
			if(c_ent.uid() != ent.uid()) continue;
			entities[j] = entities[entities.length-1];
			entities.pop();
			break;
		}
// only chop leaf nodes that don't have any entities
		// if(entities.length > 0) { continue; }
		queue.push(conflicts[i]); // push all non-intersections to queue for reprocessing
	}

	if(!queue.empty()) {
		do {
			const pop = queue.pop();
			const pt = pop.parent();
			if(!pt) continue;
			const children = pt.children();
			let ent_count = 0;
			for(let j=0;j<children.length;j++) { ent_count += children[j].entities().length; }
			if(ent_count > 0) continue;
// none of the neighbouring adjacent cells have any entities. Time to break the leaf nodes up!
			for(let j=children.length-1;j>=0;j--) children.pop();
// only tell parent to check its leaf cells (we are leaf cells now)
			if(pt == null) continue; // we've reached the root
			queue.push(pt);
		}while(!queue.empty());
	}
}

// query an AABB and find other adjacent boxes 
const QUERY_QUADTREE=(root, box, uid, adjacent=null)=> {
	if(!adjacent) adjacent = [];
	else {
		for(let i=adjacent.length-1;i>=0;i--) adjacent.pop();
	}
	const queue = new Queue();
	if(root) {
		queue.push(root);
		do {
			const pop = queue.pop();
			const children = pop.children();
			const entities = pop.entities();
			if(TEST_AABB_AABB(pop.aabb(), box)) {
				if(pop.isleaf()) {
					for(let i=0;i<entities.length;i++) {
						if(!TEST_AABB_AABB(entities[i].aabb(), box) || entities[i].uid() == uid) continue;
						adjacent.push(entities[i]);
					}
				}else {
					for(let i=0;i<children.length;i++) queue.push(children[i]);	
				}
			}
		}while(!queue.empty());
	}
	return adjacent;
}

// fixes a quadtree in the event a root's entity count is larger than defined
const EVOLVE_QUADTREE=(root)=> {
	if(!root) return;
	const entities = root.entities();
	const children = root.children();
	if(entities.length < MAX_ENTITIES_PER_LEAF) return;
	if(root.depth() > MAX_QUADTREE_DEPTH) return;
	SPLIT_QUADTREE(root);
	for(let i = entities.length-1; i>=0; i--) {
		const ent = entities[i];
		for(let j = 0; j < children.length; j++) {
			if(TEST_AABB_AABB(ent.aabb(), children[j].aabb())) {
				children[j].entities().push(ent);
			}
		}
		entities.pop();
	}
// in the (likely) event that all entities of root are contained in sub-leafs
// causing invalidation, split each quadtree.
// PROBLEM: this uses recursion. Very bad!
	for(let j = 0;j < children.length;j++) EVOLVE_QUADTREE(children[j]);
}

// splits a leaf in question into four quadrants
const SPLIT_QUADTREE=(root)=> {
	if(!root) return;
	if(!root.isleaf()) return;
	const children = root.children();
	const entities = root.entities();
	const box = root.aabb();
// split into 4 quadrants
	const split = SPLIT_AABB(box);
// place quadrants (top left, top right, bottom right, bottom left)
	for(let i=0;i < split.length;i++) { 
		const child = new QuadNode(root, split[i]);
		children.push(child);
	}
}
