#ifndef __B3_CONTACT_GRAPH_H__
#define __B3_CONTACT_GRAPH_H__

#include "..\..\Collision\b3BroadPhase.h"

class b3BlockAllocator;
class b3Contact;
class b3ContactListener;

class b3ContactGraph {
public :
	b3ContactGraph();

	// The broad-phase will notify us if ther is a potential shape pair shapes colliding.
	void AddPair(void* data1, void* data2);
	// The broad-phase will contact us if ther is a potential shape pair shapes colliding.
	void DestroyContact(b3Contact* c);
	
	// Get the potential colliding shape pairs (broad-phase).
	void FindNewContacts();
	// Get the actual colliding shapes (narrow-phase).
	void UpdateContacts();
protected :
	friend class b3Scene;
	friend class b3Body;

	b3BlockAllocator* m_blockAllocator;
	b3BroadPhase m_broadPhase;
	b3Contact* m_contactList;
	u32 m_contactCount;
	b3ContactListener* m_contactListener;
};

#endif