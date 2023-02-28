// CREDITS: Daniel J. Cucuzza
// DATE: February 19th, 2023
// You can contact me at gaunletgames@gmail.com if you have
// any questions about the implementation or if you notice
// any errors.

// constructs a default finite state entity (base class most configurations should run)
const CONSTRUCTOR_FSE=(fsm, man, init)=> {
	if(!man) man = CONSTRUCTOR_MAN();
	const ent = { fsm:fsm, man:man } // get entity
	fsm.setup(man); // invoke all setup functions
	fsm.set(man, init ? init : 'init'); // run the init state
	return ent;
}
// constructs a default man object for a FSM
const CONSTRUCTOR_MAN=()=> {
	return {
		_cur:null, // assign first state
		cur:function() { return this._cur; },
		setcur(nxt) { this._cur = nxt; },
		dt:function() { return deltaTime / 1000; }
	}
};
// we assume that our state machine is initialized and does not modify existing
// data that the fsm requires.
class FSM {
	#_dict;
// states, middleman
	constructor(states) {
		this.assert(states != null && states.length > 0, "state bag was empty or null.");
		this.#_dict = [];
// append all new states to dictionary object
		for(let i = 0;i < states.length;i++) {
			const state = states[i];
			this.vstate(state, "state object was not constructed properly. see fsm.js.");
			this.#_dict[state.key] = state;
		}
	}
	pulse=(man)=> {
		const cur = man.cur();
		cur.pulse(this, man);
	}
	setup=(man)=> {
		for(const o in this.#_dict) { 
			const stt = this.#_dict[o];
			stt.setup(this, man);
		} 
	}
	remove=(man)=> {
		for(const o in this.#_dict) {
			const stt = this.#_dict[o];
			stt.remove(this, man);
		}
	}
	cswitch=(man, next_key)=> {
		const cur = man.cur();
		const next = this.sget(next_key);
		this.assert(next != null);
		cur.exit(next_key, this, man); 		// Notify old state of man that its leaving
		man.setcur(next);					// Context switch
		next.enter(cur.key, this, man);		// Notify new state of man that its entering
	}
	set=(man, next_key)=> {
		const next = this.sget(next_key);
		this.assert(next != null);	
		man.setcur(next);					// Context switch
		next.enter('set', this, man);		// Notify new state of man that its entering
	}
	sget=(key)=> key in this.#_dict ? this.#_dict[key] : null;
	assert(cond, output) { if(!cond) throw new Error("assertion failed:" + output); }
	vstate(state) { // determine if new state object has the required components
		return Object.hasOwn(state, 'key') &&
			Object.hasOwn(state, 'enter') &&
			Object.hasOwn(state, 'exit') &&
			Object.hasOwn(state, 'setup') &&
			Object.hasOwn(state, 'pulse');
	}
}

// simple list of objects that can be overridden after destructing.
class ObjectList {
	#_objs; #_uidh;
	constructor(uidh, nullobj) {
		this.#_uidh = uidh;
		this.#_objs = new Array();
// reserve the first slot for the null object
		this.#_uidh.reserve();
		this.#_objs.push(nullobj);
	}
	write_obj=(ctor, props)=> {
		const obj = ctor();
		const next = this.#_uidh.reserve();
// if our next index is larger, push. if not, overwrite.
		if(next >= this.#_objs.length) this.#_objs.push(obj);
		else this.#_objs[next] = obj;
// write ID
		props.id = next;
		obj.bind(props); // dependency injection
		return obj;
	}
	get_obj=(uid)=> {	
// if requested UID is zero: return null
		if(uid==0) return null;
// if the entity in question houses a zero uid, that means its dead: return null		
		const obj = this.#_objs[uid];
		if(obj.uid() == 0) return null;
		else return obj;
	}
	rem_obj=(uid, dtor)=> {
// if attempting to remove null entity, dont!
		if(uid==0) return;
		dtor(this.#_objs[uid]);
		this.#_uidh.open(uid);
	}
	length=()=> { return this.#_objs.length; }
	count=()=> { return this.length() - this.#_uidh.count(); }
// primarily useful to expose the list to the renderer. terrible idea btw.
	data=()=> { return this.#_objs; }
}
// handles assigning unique ids to every entity.
class UIDHandler {
	#_list; #_top;
	constructor() {
		this.#_list = new Array();
// any index at zero is an invalid index.
		this.#_top  = 0;
	}
// get a new id.
	reserve=()=> {
		if(this.#_list.length > 0) {
			return this.#_list.pop();
		}else {
			return this.#_top++;
		}
	}
// open up a new slot to assign to.
	open=(id)=> {
		this.#_list.push(id);
	}
// reserved # of IDs
	count=()=> { return this.#_list.length; }
}