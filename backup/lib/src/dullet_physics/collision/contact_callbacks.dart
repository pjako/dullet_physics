part of dullet_physics;



typedef void ContactCallback(WebGLPhysicsCollisionObject objectA,
                               WebGLPhysicsCollisionObject objectB,
                               List<WebGLPhysicsPublicContact> contact);

class WebGLPhysicsContactCallbacks {
    int mask;
    bool added;
    bool deferred;
    bool trigger;

    ContactCallback onPreSolveContact;


    ContactCallback onAddedContacts;
    ContactCallback onProcessedContacts;
    ContactCallback onRemovedContacts;

    WebGLPhysicsContactCallbacks({int mask, int contactCallbacksMask, bool trigger: false, ContactCallback onAddedContacts, ContactCallback onProcessedContacts, ContactCallback onRemovedContacts})
    {
        this.mask = (contactCallbacksMask != null ? contactCallbacksMask : mask);
        this.added = false;
        this.deferred = (onAddedContacts || onProcessedContacts || onRemovedContacts);
        this.onPreSolveContact = onPreSolveContact;// || null;
        this.onAddedContacts = onAddedContacts;// || null;
        this.onProcessedContacts = onProcessedContacts;// || null;
        this.onRemovedContacts = onRemovedContacts;// || null;
        this.trigger = trigger;// || false;
    }
}